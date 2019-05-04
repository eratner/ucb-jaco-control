#ifndef UCB_JACO_CONTROL_PD_COMPUTED_TORQUE_CONTROLLER_H
#define UCB_JACO_CONTROL_PD_COMPUTED_TORQUE_CONTROLLER_H

#include <ucb_jaco_control/computed_torque_controller.h>

namespace ucb_jaco_control
{

template <unsigned int Dim>
class PDComputedTorqueController : public ComputedTorqueController<Dim>
{
public:
  typedef typename ComputedTorqueController<Dim>::StateVector   StateVector;
  typedef typename ComputedTorqueController<Dim>::ControlVector ControlVector;

  PDComputedTorqueController(const std::array<double, Dim>& proportional_gain,
                             const std::array<double, Dim>& derivative_gain,
                             RobotDynamics<Dim>*            dynamics,
                             Trajectory<Dim>*               desired_trajectory)
    : ComputedTorqueController<Dim>(dynamics, desired_trajectory)
  {
    setProportionalGain(proportional_gain);
    setDerivativeGain(derivative_gain);
  }

  void setProportionalGain(const std::array<double, Dim>& gain)
  {
    Eigen::Matrix<double, Dim, 1>& diag = proportional_gain_.diagonal();
    for (int i = 0; i < Dim; ++i)
      diag(i) = gain[i];
  }

  void setDerivativeGain(const std::array<double, Dim>& gain)
  {
    Eigen::Matrix<double, Dim, 1>& diag = derivative_gain_.diagonal();
    for (int i = 0; i < Dim; ++i)
      diag(i) = gain[i];
  }

  const Eigen::DiagonalMatrix<double, Dim>& getProportionalGainMatrix() const
  {
    return proportional_gain_;
  }

  const Eigen::DiagonalMatrix<double, Dim>& getDerivativeGainMatrix() const
  {
    return derivative_gain_;
  }

protected:
  ControlVector getOuterLoopControl(const StateVector& state, double t)
  {
    // Get the current position and velocities from the state.
    Eigen::Matrix<double, Dim, 1> pos = state.block(0, 0, Dim, 1);
    Eigen::Matrix<double, Dim, 1> vel = state.block(Dim, 0, Dim, 1);

    Eigen::Matrix<double, Dim, 1> err       = this->desired_trajectory_->getPosition(t) - pos;
    Eigen::Matrix<double, Dim, 1> err_deriv = this->desired_trajectory_->getVelocity(t) - vel;

    // Implement a PD control law.
    return -1.0 * proportional_gain_ * err - derivative_gain_ * err_deriv;
  }

  Eigen::DiagonalMatrix<double, Dim> proportional_gain_; // Proportional gain matrix.
  Eigen::DiagonalMatrix<double, Dim> derivative_gain_;   // Derivative gain matrix.

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_PD_COMPUTED_TORQUE_CONTROLLER_H
