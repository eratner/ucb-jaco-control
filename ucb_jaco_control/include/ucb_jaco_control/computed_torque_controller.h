#ifndef UCB_JACO_CONTROL_COMPUTED_TORQUE_CONTROLLER_H
#define UCB_JACO_CONTROL_COMPUTED_TORQUE_CONTROLLER_H

#include <ucb_jaco_control/state_feedback_controller.h>
#include <ucb_jaco_control/trajectory.h>
#include <ucb_jaco_control/robot_dynamics.h>

namespace ucb_jaco_control
{

template <unsigned int Dim>
class ComputedTorqueController : public StateFeedbackController<2*Dim, Dim>
{
public:
  typedef Eigen::Matrix<double, 2*Dim, 1> StateVector;
  typedef Eigen::Matrix<double, Dim, 1>   ControlVector;

  ComputedTorqueController(RobotDynamics<Dim>* dynamics,
                           Trajectory<Dim>*    desired_trajectory)
    : dynamics_(dynamics), desired_trajectory_(desired_trajectory)
  {
  }

  virtual ~ComputedTorqueController()
  {
  }

  ControlVector getControl(const StateVector& state, double t)
  {
    return getInnerLoopControl(state, t);
  }

  Trajectory<Dim>* getDesiredTrajectory()
  {
    return desired_trajectory_;
  }

  void setDesiredTrajectory(Trajectory<Dim>* desired_trajectory)
  {
    desired_trajectory_ = desired_trajectory;
  }

protected:
  virtual ControlVector getInnerLoopControl(const StateVector& state, double t)
  {
    Eigen::Matrix<double, Dim, 1> des_acc = desired_trajectory_->getAcceleration(t);
    ControlVector u = getOuterLoopControl(state, t);

    // Get the current position and velocities from the state.
    Eigen::Matrix<double, Dim, 1> pos = state.block(0, 0, Dim, 1);
    Eigen::Matrix<double, Dim, 1> vel = state.block(Dim, 0, Dim, 1);

    Eigen::Matrix<double, Dim, 1> N = dynamics_->getCoriolisVector(pos, vel)
      + dynamics_->getGravityVector(pos)
      + dynamics_->getFrictionVector(pos);

    typename RobotDynamics<Dim>::Matrix M = dynamics_->getInertiaMatrix(pos);

    return M * (des_acc - u) + N;
  }

  virtual ControlVector getOuterLoopControl(const StateVector& state, double t) = 0;

  RobotDynamics<Dim>* dynamics_;
  Trajectory<Dim>*    desired_trajectory_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_COMPUTED_TORQUE_CONTROLLER_H
