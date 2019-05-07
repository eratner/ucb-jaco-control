#ifndef UCB_JACO_CONTROL_PID_COMPUTED_TORQUE_CONTROLLER_H
#define UCB_JACO_CONTROL_PID_COMPUTED_TORQUE_CONTROLLER_H

#include <ucb_jaco_control/computed_torque_controller.h>
#include <angles/angles.h>

namespace ucb_jaco_control
{

template <unsigned int Dim>
class PIDComputedTorqueController : public ComputedTorqueController<Dim>
{
public:
  typedef typename ComputedTorqueController<Dim>::StateVector   StateVector;
  typedef typename ComputedTorqueController<Dim>::ControlVector ControlVector;

  PIDComputedTorqueController(const std::array<double, Dim>& proportional_gain,
                              const std::array<double, Dim>& integral_gain,
                              const std::array<double, Dim>& derivative_gain,
                              RobotDynamics<Dim>*            dynamics,
                              Trajectory<Dim>*               desired_trajectory)
    : ComputedTorqueController<Dim>(dynamics, desired_trajectory)
  {
    setProportionalGain(proportional_gain);
    setIntegralGain(integral_gain);
    setDerivativeGain(derivative_gain);
  }

  void setProportionalGain(const std::array<double, Dim>& gain)
  {
    Eigen::Matrix<double, Dim, 1>& diag = proportional_gain_.diagonal();
    for (int i = 0; i < Dim; ++i)
      diag(i) = gain[i];
  }

  void setIntegralGain(const std::array<double, Dim>& gain)
  {
    Eigen::Matrix<double, Dim, 1>& diag = integral_gain_.diagonal();
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

  const Eigen::DiagonalMatrix<double, Dim>& getIntegralGainMatrix() const
  {
    return integral_gain_;
  }

  const Eigen::DiagonalMatrix<double, Dim>& getDerivativeGainMatrix() const
  {
    return derivative_gain_;
  }

  void reset()
  {
    // Set integral of error to zero.
    integral_ = Eigen::Matrix<double, Dim, 1>::Zero();

    // Setting the time to < 0 will cause it to reset the next time a control is requested.
    last_time_ = -1.0;
  }

  const Eigen::Matrix<double, Dim, 1> &getError() const
  {
    return error_;
  }

protected:
  ControlVector getOuterLoopControl(const StateVector& state, double t)
  {
    // Get the current position and velocities from the state.
    Eigen::Matrix<double, Dim, 1> pos = state.block(0, 0, Dim, 1);
    Eigen::Matrix<double, Dim, 1> vel = state.block(Dim, 0, Dim, 1);

    Eigen::Matrix<double, Dim, 1> err       = this->desired_trajectory_->getPosition(t) - pos;
    Eigen::Matrix<double, Dim, 1> err_deriv = this->desired_trajectory_->getVelocity(t) - vel;

    // Update the integral of the error.
    if (last_time_ < 0)
      last_time_ = t;

    double dt = t - last_time_;
    last_time_ = t;
    integral_ += dt * err;

    error_ = err;

    // Implement a PID control law.
    return -1.0 * proportional_gain_ * err -
      integral_gain_ * integral_ -
      derivative_gain_ * err_deriv;
  }

  Eigen::DiagonalMatrix<double, Dim> proportional_gain_; // Proportional gain matrix.
  Eigen::DiagonalMatrix<double, Dim> integral_gain_;     // Integral gain matrix.
  Eigen::DiagonalMatrix<double, Dim> derivative_gain_;   // Derivative gain matrix.

  Eigen::Matrix<double, Dim, 1>      integral_;          // Integral of the error.
  double                             last_time_;         // Last time a control was requested.
  Eigen::Matrix<double, Dim, 1>      error_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_PID_COMPUTED_TORQUE_CONTROLLER_H
