#ifndef UCB_JACO_CONTROL_PID_REGULATION_CONTROLLER_H
#define UCB_JACO_CONTROL_PID_REGULATION_CONTROLLER_H

#include <ucb_jaco_control/state_feedback_controller.h>
#include <array>

namespace ucb_jaco_control
{

template <unsigned int StateDim>
class PIDRegulationController : public StateFeedbackController<StateDim, StateDim>
{
public:
  using typename StateFeedbackController<StateDim, StateDim>::StateVector;
  using typename StateFeedbackController<StateDim, StateDim>::ControlVector;

  PIDRegulationController(const std::array<double, StateDim>& proportional_gain,
                          const std::array<double, StateDim>& derivative_gain,
                          const std::array<double, StateDim>& integral_gain,
                          const StateVector& setpoint = StateVector::Zero())
    : setpoint_(setpoint)
  {
    setProportionalGain(proportional_gain);
    setDerivativeGain(derivative_gain);
    setIntegralGain(integral_gain);
    reset();
  }

  ~PIDRegulationController()
  {
  }

  void setProportionalGain(const std::array<double, StateDim>& gain)
  {
    Eigen::Matrix<double, StateDim, 1>& diag = proportional_gain_.diagonal();
    for (int i = 0; i < StateDim; ++i)
      diag(i) = gain[i];
  }

  void setDerivativeGain(const std::array<double, StateDim>& gain)
  {
    Eigen::Matrix<double, StateDim, 1>& diag = derivative_gain_.diagonal();
    for (int i = 0; i < StateDim; ++i)
      diag(i) = gain[i];
  }

  void setIntegralGain(const std::array<double, StateDim>& gain)
  {
    Eigen::Matrix<double, StateDim, 1>& diag = integral_gain_.diagonal();
    for (int i = 0; i < StateDim; ++i)
      diag(i) = gain[i];
  }

  const Eigen::DiagonalMatrix<double, StateDim>& getProportionalGainMatrix() const
  {
    return proportional_gain_;
  }

  const Eigen::DiagonalMatrix<double, StateDim>& getDerivativeGainMatrix() const
  {
    return derivative_gain_;
  }

  const Eigen::DiagonalMatrix<double, StateDim>& getIntegralGainMatrix() const
  {
    return integral_gain_;
  }

  const StateVector& getSetpoint() const
  {
    return setpoint_;
  }

  void setSetpoint(const StateVector& setpoint)
  {
    setpoint_ = setpoint;
  }

  void reset()
  {
    // Set integral to zero.
    integral_ = StateVector::Zero();

    // Clear last error.
    last_error_ = StateVector::Zero();
  }

  const StateVector& getError() const
  {
    return last_error_;
  }

  ControlVector getControl(const StateVector& state, double dt)
  {
    // Compute the error.
    StateVector error = setpoint_ - state;

    // Update the integral.
    integral_ += dt * error;

    ControlVector control = proportional_gain_ * error -
      derivative_gain_ * ((error - last_error_) / dt) -
      integral_gain_ * integral_;

    // Update the last error.
    last_error_ = error;

    return control;
  }

protected:
  Eigen::DiagonalMatrix<double, StateDim> proportional_gain_; // Proportional gain matrix.
  Eigen::DiagonalMatrix<double, StateDim> derivative_gain_;   // Derivative gain matrix.
  Eigen::DiagonalMatrix<double, StateDim> integral_gain_;     // Integral gain matrix.

  StateVector                             setpoint_;          // Setpoint to regulate to.

  StateVector                             integral_;
  StateVector                             last_error_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_PID_REGULATION_CONTROLLER_H
