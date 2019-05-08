#include <ros/ros.h>
#include <ucb_jaco_control/urdf_robot_dynamics.h>
#include <ucb_jaco_control/runge_kutta.h>
#include <ucb_jaco_control/pid_computed_torque_controller.h>
#include <ucb_jaco_control/constant_trajectory.h>
#include <sensor_msgs/JointState.h>

namespace ucb_jaco_control
{

class JacoSimulatorROS
{
public:
  static const unsigned int DOF = 7;

  typedef Eigen::Matrix<double, 2*DOF, 1> StateVector;
  typedef Eigen::Matrix<double, DOF, 1> ControlVector;

  JacoSimulatorROS()
    : private_node_handle_("~"), loop_rate_(100.0), controller_(nullptr)
  {
    std::string urdf_file = private_node_handle_.param<std::string>("urdf_file",
                                                                    "/home/eratner/j2s7s300.urdf");
    std::string root_name = private_node_handle_.param<std::string>("root_name",
                                                                    "j2s7s300_link_base");
    std::string tip_name  = private_node_handle_.param<std::string>("tip_name",
                                                                    "j2s7s300_end_effector");

    dynamics_ = new URDFRobotDynamics<DOF>(urdf_file,
                                           root_name,
                                           tip_name);

    std::function<StateVector (double, const StateVector&, const ControlVector&)> f =
      [&] (double t, const StateVector& state, const ControlVector& control)
    {
      Eigen::Matrix<double, DOF, 1> pos = state.block(0, 0, DOF, 1);
      Eigen::Matrix<double, DOF, 1> vel = state.block(DOF, 0, DOF, 1);

      Eigen::Matrix<double, DOF, DOF> M_inv = dynamics_->getInertiaMatrix(pos).inverse();

      Eigen::Matrix<double, DOF, 1> acc = M_inv * (control -
                                                   dynamics_->getCoriolisVector(pos, vel) -
                                                   dynamics_->getGravityVector(pos) -
                                                   dynamics_->getFrictionVector(vel));

      StateVector state_deriv;
      state_deriv.block(0, 0, DOF, 1) = vel;
      state_deriv.block(DOF, 0, DOF, 1) = acc;
      return state_deriv;
    };

    integrator_ = new RungeKutta<2*DOF, DOF>(f, 1. / loop_rate_);

    joint_state_pub_ = private_node_handle_.advertise<sensor_msgs::JointState>("joint_states",
                                                                               100);
  }

  ~JacoSimulatorROS()
  {
    delete dynamics_;
    dynamics_ = nullptr;

    delete integrator_;
    integrator_ = nullptr;

    delete controller_;
    controller_ = nullptr;
  }

  void run()
  {
    ros::Rate rate(loop_rate_);

    // Initialize the robot's state.
    state_ = StateVector::Zero();

    // Initialize the controller.
    Eigen::Matrix<double, DOF, 1> setpoint;
    setpoint << 0, M_PI, 0, M_PI, 0, M_PI, 0;
    ConstantTrajectory<DOF>* desired_trajectory = new ConstantTrajectory<DOF>(setpoint);

    std::array<double, DOF> p_gain;
    p_gain.fill(100.0);

    std::array<double, DOF> i_gain;
    i_gain.fill(0.0);

    std::array<double, DOF> d_gain;
    d_gain.fill(20.0);

    controller_ = new PIDComputedTorqueController<DOF>(p_gain,
                                                       i_gain,
                                                       d_gain,
                                                       dynamics_,
                                                       desired_trajectory);

    std::function<ControlVector (double, StateVector)> control = [&] (double t, StateVector state)
    {
      return controller_->getControl(state, t);
      // return ControlVector::Zero();
    };

    double start_t = ros::Time::now().toSec();

    while (ros::ok())
    {
      double t = ros::Time::now().toSec() - start_t;

      // Integrate to get the next state.
      state_ = integrator_->solve(state_, control, t);

      // Publish the joint states message.
      sensor_msgs::JointState msg;
      msg.header.stamp = ros::Time::now();
      msg.name = {"j2s7s300_joint_1",
                  "j2s7s300_joint_2",
                  "j2s7s300_joint_3",
                  "j2s7s300_joint_4",
                  "j2s7s300_joint_5",
                  "j2s7s300_joint_6",
                  "j2s7s300_joint_7",
                  "j2s7s300_joint_finger_1",
                  "j2s7s300_joint_finger_2",
                  "j2s7s300_joint_finger_3"};
      msg.position = {state_(0),
                      state_(1),
                      state_(2),
                      state_(3),
                      state_(4),
                      state_(5),
                      state_(6),
                      0.,
                      0.,
                      0.};
      msg.velocity = {state_(7),
                      state_(8),
                      state_(9),
                      state_(10),
                      state_(11),
                      state_(12),
                      state_(13),
                      0.,
                      0.,
                      0.};
      joint_state_pub_.publish(msg);

      rate.sleep();
      ros::spinOnce();
    }

    delete desired_trajectory;
    desired_trajectory = nullptr;
  }

private:
  ros::NodeHandle                   private_node_handle_;
  ros::NodeHandle                   node_handle_;

  ros::Publisher                    joint_state_pub_;
  double                            loop_rate_;

  URDFRobotDynamics<DOF>*           dynamics_;
  RungeKutta<2*DOF, DOF>*           integrator_;
  StateVector                       state_;

  PIDComputedTorqueController<DOF>* controller_;

};

} // namespace ucb_jaco_control

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jaco_simulator");

  ucb_jaco_control::JacoSimulatorROS simulator;
  simulator.run();

  return 0;
}
