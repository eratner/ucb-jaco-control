#ifndef UCB_JACO_CONTROL_PID_REGULATION_CONTROLLER_H
#define UCB_JACO_CONTROL_PID_REGULATION_CONTROLLER_H

#include <ucb_jaco_control/state_feedback_controller.h>
#include <array>
#include <angles/angles.h> // TODO: Would be nice not to depend on ROS packages here.

namespace ucb_jaco_control
{

template <unsigned int StateDim>
class PIDRegulationController : public StateFeedbackController<2*StateDim, StateDim>
{
public:
  typedef Eigen::Matrix<double, StateDim, 1>   StateVector;
  typedef Eigen::Matrix<double, 2*StateDim, 1> AugmentedStateVector;
  typedef Eigen::Matrix<double, StateDim, 1>   ControlVector;

  PIDRegulationController(const std::array<double, StateDim>& proportional_gain,
                          const std::array<double, StateDim>& integral_gain,
                          const std::array<double, StateDim>& derivative_gain,
                          const StateVector& setpoint = StateVector::Zero(),
                          bool wrap_angles = false)
    : setpoint_(setpoint)
  {
    setProportionalGain(proportional_gain);
    setIntegralGain(integral_gain);
    setDerivativeGain(derivative_gain);
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

  void setIntegralGain(const std::array<double, StateDim>& gain)
  {
    Eigen::Matrix<double, StateDim, 1>& diag = integral_gain_.diagonal();
    for (int i = 0; i < StateDim; ++i)
      diag(i) = gain[i];
  }

  void setDerivativeGain(const std::array<double, StateDim>& gain)
  {
    Eigen::Matrix<double, StateDim, 1>& diag = derivative_gain_.diagonal();
    for (int i = 0; i < StateDim; ++i)
      diag(i) = gain[i];
  }

  const Eigen::DiagonalMatrix<double, StateDim>& getProportionalGainMatrix() const
  {
    return proportional_gain_;
  }

  const Eigen::DiagonalMatrix<double, StateDim>& getIntegralGainMatrix() const
  {
    return integral_gain_;
  }

  const Eigen::DiagonalMatrix<double, StateDim>& getDerivativeGainMatrix() const
  {
    return derivative_gain_;
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

  ControlVector getControl(const AugmentedStateVector& state, double dt)
  {
    StateVector pos = state.block(0, 0, StateDim, 1);
    StateVector vel = state.block(StateDim, 0, StateDim, 1);

    // Compute the error and derivative.
    StateVector err       = diff(setpoint_, pos);
    StateVector err_deriv = -vel;

    // Update the integral.
    integral_ += dt * err;

    ControlVector control = -1.0 * proportional_gain_ * err
      - derivative_gain_ * err_deriv
      - integral_gain_ * integral_;

    // Update the last error.
    last_error_ = err;

    return control;
  }

protected:
  StateVector diff(const StateVector &a, const StateVector &b) const
  {
    StateVector d;
    if (wrap_angles_)
    {
      for (int i = 0; i < StateDim; ++i)
        d(i) = angles::shortest_angular_distance(a(i), b(i));
    }
    else
      d = b - a;

    return d;
  }

  Eigen::DiagonalMatrix<double, StateDim> proportional_gain_; // Proportional gain matrix.
  Eigen::DiagonalMatrix<double, StateDim> derivative_gain_;   // Derivative gain matrix.
  Eigen::DiagonalMatrix<double, StateDim> integral_gain_;     // Integral gain matrix.

  StateVector                             setpoint_;          // Setpoint to regulate to.
  bool                                    wrap_angles_;       // If set, wraps values to [-pi, pi).

  StateVector                             integral_;
  StateVector                             last_error_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_PID_REGULATION_CONTROLLER_H
