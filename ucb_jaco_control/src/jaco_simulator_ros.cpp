#include <ros/ros.h>
#include <ucb_jaco_control/urdf_robot_dynamics.h>
#include <ucb_jaco_control/runge_kutta.h>
#include <ucb_jaco_control/pid_computed_torque_controller.h>
#include <ucb_jaco_control/constant_trajectory.h>
#include <ucb_jaco_control/sinusoidal_trajectory.h>
#include <ucb_jaco_control/buffer.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_reconfigure/server.h>
#include <ucb_jaco_control/DynamicsModelParamsConfig.h>

#include <extern/matplotlibcpp/matplotlibcpp.h>

namespace plt = matplotlibcpp;

namespace ucb_jaco_control
{

class JacoSimulatorROS
{
public:
  static const unsigned int DOF = 7;
  static constexpr const double joint_zero_offset[DOF] = {0., M_PI, 0., M_PI, 0., M_PI, 0.};

  typedef Eigen::Matrix<double, 2*DOF, 1> StateVector;
  typedef Eigen::Matrix<double, DOF, 1> ControlVector;

  JacoSimulatorROS()
    : private_node_handle_("~"), loop_rate_(100.0), controller_(nullptr), buffer_(1000)
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

    controller_dynamics_ = new URDFRobotDynamics<DOF>(urdf_file,
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

    // Dynamic reconfigure for the dynamics model parameters.
    server_ = new dynamic_reconfigure::Server<DynamicsModelParamsConfig>(private_node_handle_);
    server_->setCallback(boost::bind(&JacoSimulatorROS::dynamicsModelParamsCallback, this, _1, _2));
  }

  ~JacoSimulatorROS()
  {
    delete dynamics_;
    dynamics_ = nullptr;

    delete controller_dynamics_;
    controller_dynamics_ = nullptr;

    delete integrator_;
    integrator_ = nullptr;

    delete controller_;
    controller_ = nullptr;

    delete server_;
    server_ = nullptr;
  }

  void dynamicsModelParamsCallback(DynamicsModelParamsConfig &config, uint32_t level)
  {
    std::vector<std::string> link_names =
    {
      "j2s7s300_link_1",
      "j2s7s300_link_2",
      "j2s7s300_link_3",
      "j2s7s300_link_4",
      "j2s7s300_link_5",
      "j2s7s300_link_6",
      "j2s7s300_link_7"
    };

    std::vector<double> masses =
    {
      config.link_1_mass,
      config.link_2_mass,
      config.link_3_mass,
      config.link_4_mass,
      config.link_5_mass,
      config.link_6_mass,
      config.link_7_mass
    };

    try
    {
      dynamics_->setLinkMasses(link_names, masses);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("Error: " << e.what());
    }
  }

  void run()
  {
    ros::Rate rate(loop_rate_);

    // Initialize the robot's state.
    state_ = StateVector::Zero();
    state_(0) = joint_zero_offset[0];
    state_(1) = joint_zero_offset[1] + M_PI_4;
    state_(2) = joint_zero_offset[2];
    state_(3) = joint_zero_offset[3] + M_PI / 8.;
    state_(4) = joint_zero_offset[4];
    state_(5) = joint_zero_offset[5];
    state_(6) = joint_zero_offset[6];

    // Initialize the controller.
    // Eigen::Matrix<double, DOF, 1> setpoint;
    // setpoint << 0, M_PI, 0, M_PI, 0, M_PI, 0;
    // ConstantTrajectory<DOF>* desired_trajectory = new ConstantTrajectory<DOF>(setpoint);
    SinusoidalTrajectory<DOF>* desired_trajectory =
      new SinusoidalTrajectory<DOF>({0., 0.8, 0., 1.0, 0., 0.8, 0.},
        {0., 0.8, 0., 0.8, 0., 0.8, 0.},
        {0., 0., 0., 0., 0., 0., 0.},
        {joint_zero_offset[0],
            joint_zero_offset[1],
            joint_zero_offset[2],
            joint_zero_offset[3],
            joint_zero_offset[4],
            joint_zero_offset[5],
            joint_zero_offset[6]});

    std::array<double, DOF> p_gain;
    p_gain.fill(100.0);

    std::array<double, DOF> i_gain;
    i_gain.fill(0.0);

    std::array<double, DOF> d_gain;
    d_gain.fill(20.0);

    controller_ = new PIDComputedTorqueController<DOF>(p_gain,
                                                       i_gain,
                                                       d_gain,
                                                       controller_dynamics_,
                                                       desired_trajectory);

    std::function<ControlVector (double, StateVector)> control = [&] (double t, StateVector state)
    {
      return controller_->getControl(state, t);
      // return ControlVector::Zero();
    };

    joint_state_pub_.publish(stateToJointMsg(state_));

    ROS_INFO("Waiting 5 s before starting...");
    ros::Duration(5.0).sleep();
    ROS_INFO("...okay, go!");

    double start_t = ros::Time::now().toSec();

    while (ros::ok())
    {
      double t = ros::Time::now().toSec() - start_t;

      // For logging, get the next control.
      ControlVector u = control(t, state_);

      // Integrate to get the next state.

      state_ = integrator_->solve(state_, control, t);

      // Record the data.
      Data data;
      data.t = t;
      data.pos = {state_(0), state_(1), state_(2), state_(3), state_(4), state_(5), state_(6)};
      data.vel = {state_(7), state_(8), state_(9), state_(10), state_(11), state_(12), state_(13)};
      Eigen::Matrix<double, DOF, 1> des_pos = desired_trajectory->getPosition(t);
      data.des_pos =
        {des_pos(0), des_pos(1), des_pos(2), des_pos(3), des_pos(4), des_pos(5), des_pos(6)};
      Eigen::Matrix<double, DOF, 1> des_vel = desired_trajectory->getVelocity(t);
      data.des_vel =
        {des_vel(0), des_vel(1), des_vel(2), des_vel(3), des_vel(4), des_vel(5), des_vel(6)};
      data.u = {u(0), u(1), u(2), u(3), u(4), u(5), u(6)};
      // Hacky, but records data until the buffer is full.
      if (buffer_.count() < buffer_.capacity())
        buffer_.add(data);

      // Publish the joint states message.
      joint_state_pub_.publish(stateToJointMsg(state_));

      rate.sleep();
      ros::spinOnce();
    }

    delete desired_trajectory;
    desired_trajectory = nullptr;
  }

  sensor_msgs::JointState stateToJointMsg(const StateVector &state)
  {
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
    msg.position = {state(0),
                    state(1),
                    state(2),
                    state(3),
                    state(4),
                    state(5),
                    state(6),
                    0.,
                    0.,
                    0.};
    msg.velocity = {state(7),
                    state(8),
                    state(9),
                    state(10),
                    state(11),
                    state(12),
                    state(13),
                    0.,
                    0.,
                    0.};
    return msg;
  }

  void plot()
  {
    ROS_INFO("Plotting...");
    std::vector<Data> data = buffer_.getData();

    // Plot the actual and desired positions.
    plt::figure(1);
    plt::figure_size(1200, 800);
    plt::title("Joint Positions");

    for (int i = 0; i < DOF; ++i)
    {
      std::vector<double> pos;
      std::vector<double> des_pos;
      std::vector<double> times;

      for (const Data& d : data)
      {
        pos.push_back(d.pos[i]);
        des_pos.push_back(d.des_pos[i]);
        times.push_back(d.t);
      }

      plt::named_plot("q" + std::to_string(i), times, pos);
      plt::named_plot("q" + std::to_string(i) + " (desired)", times, des_pos, "--");
    }

    plt::save("/home/eratner/catkin_ws/src/ucb-jaco-control/joint_positions.png");
    plt::legend();
    plt::save("/home/eratner/catkin_ws/src/ucb-jaco-control/joint_positions_legend.png");

    // Plot the actual and desired velocities.
    plt::figure(2);
    plt::figure_size(1200, 800);
    plt::title("Joint Velocities");

    for (int i = 0; i < DOF; ++i)
    {
      std::vector<double> vel;
      std::vector<double> des_vel;
      std::vector<double> times;

      for (const Data& d : data)
      {
        vel.push_back(d.vel[i]);
        des_vel.push_back(d.des_vel[i]);
        times.push_back(d.t);
      }

      plt::named_plot("v" + std::to_string(i), times, vel);
      plt::named_plot("v" + std::to_string(i) + " (desired)", times, des_vel, "--");
    }

    plt::save("/home/eratner/catkin_ws/src/ucb-jaco-control/joint_velocities.png");
    plt::legend();
    plt::save("/home/eratner/catkin_ws/src/ucb-jaco-control/joint_velocities_legend.png");

    // Plot the control inputs.
    plt::figure(3);
    plt::figure_size(1200, 800);
    plt::title("Joint Torques");

    for (int i = 0; i < DOF; ++i)
    {
      std::vector<double> u;
      std::vector<double> times;

      for (const Data& d : data)
      {
        u.push_back(d.u[i]);
        times.push_back(d.t);
      }

      plt::named_plot("u" + std::to_string(i), times, u);
    }

    plt::save("/home/eratner/catkin_ws/src/ucb-jaco-control/control_effort.png");
    plt::legend();
    plt::save("/home/eratner/catkin_ws/src/ucb-jaco-control/control_effort_legend.png");

    // plt::show();
    ROS_INFO("...done.");
  }

private:
  struct Data
  {
    std::array<double, DOF> pos;
    std::array<double, DOF> des_pos;
    std::array<double, DOF> vel;
    std::array<double, DOF> des_vel;
    std::array<double, DOF> u;
    double                  t;
  };

  ros::NodeHandle                                        private_node_handle_;
  ros::NodeHandle                                        node_handle_;

  ros::Publisher                                         joint_state_pub_;
  double                                                 loop_rate_;

  dynamic_reconfigure::Server<DynamicsModelParamsConfig>* server_;

  URDFRobotDynamics<DOF>*                                dynamics_;
  URDFRobotDynamics<DOF>*                                controller_dynamics_;
  RungeKutta<2*DOF, DOF>*                                integrator_;
  StateVector                                            state_;

  PIDComputedTorqueController<DOF>*                      controller_;
  Buffer<Data>                                           buffer_;

};

} // namespace ucb_jaco_control

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "jaco_simulator");

  ucb_jaco_control::JacoSimulatorROS simulator;
  simulator.run();
  simulator.plot();

  return 0;
}
