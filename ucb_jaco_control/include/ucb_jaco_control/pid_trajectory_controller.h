#ifndef UCB_JACO_CONTROL_PID_TRAJECTORY_CONTROLLER_H
#define UCB_JACO_CONTROL_PID_TRAJECTORY_CONTROLLER_H

#include <ucb_jaco_control/state_feedback_controller.h>
#include <ucb_jaco_control/trajectory.h>
#include <array>
#include <angles/angles.h>

namespace ucb_jaco_control
{

template <unsigned int StateDim>
class PIDTrajectoryController : public StateFeedbackController<2*StateDim, StateDim>
{
public: 
  using typename StateFeedbackController<2*StateDim, StateDim>::StateVector;
  using typename StateFeedbackController<2*StateDim, StateDim>::ControlVector;

  PIDTrajectoryController(const std::array<double, StateDim>& proportional_gain,
                          const std::array<double, StateDim>& integral_gain,
                          const std::array<double, StateDim>& derivative_gain,
                          const ucb_jaco_control::Trajectory<StateDim>& desired_trajectory,
                          bool wrap_angles = false)
    : desired_trajectory_(desired_trajectory)
  {
    setProportionalGain(proportional_gain);
    setDerivativeGain(derivative_gain);
    setIntegralGain(integral_gain);
    reset();
  }

  ~PIDTrajectoryController()
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

  void reset()
  {
    // Set integral to zero.
    integral_ = Eigen::Matrix<double, StateDim, 1>::Zero();

    // Set time back to 0
    previous_time_ = 0;
  }

  ControlVector getControl(const StateVector& state, double t)
  {
    Eigen::Matrix<double, StateDim, 1> curr_pos = state.block(0, 0, StateDim, 1);
    Eigen::Matrix<double, StateDim, 1> curr_vel = state.block(StateDim, 0, StateDim, 1);

    // calculate dt
    // TODO: handle 0 dt somehow
    double dt = t - previous_time_;

    // get next point on desired trajectory
    Eigen::Matrix<double, StateDim, 1> des_pos = desired_trajectory_.getDesiredPosition(t);
    Eigen::Matrix<double, StateDim, 1> des_vel = desired_trajectory_.getDesiredVelocity(t);
    
    // update time
    previous_time_ = t;

    // Compute the error.
    Eigen::Matrix<double, StateDim, 1> error = diff(des_pos, curr_pos);

    //Computer error derivative
    Eigen::Matrix<double, StateDim, 1> error_der = diff(des_vel, curr_vel);

    // Update the integral.
    integral_ += dt * error;

    ControlVector control = -1 * proportional_gain_ * error -
      derivative_gain_ * error_der -
      integral_gain_ * integral_;

    return control;
  }

  protected:
    Eigen::Matrix<double, StateDim, 1> diff(const Eigen::Matrix<double, StateDim, 1> &a, const Eigen::Matrix<double, StateDim, 1> &b) const
    {
      Eigen::Matrix<double, StateDim, 1> d;
      if (wrap_angles_)
      {
        for (int i = 0; i < StateDim; ++i)
          d(i) = angles::shortest_angular_distance(a(i), b(i));
      }
      else
        d = b - a;

      return d;
    }
    Eigen::DiagonalMatrix<double, StateDim>       proportional_gain_;
    Eigen::DiagonalMatrix<double, StateDim>       derivative_gain_;
    Eigen::DiagonalMatrix<double, StateDim>       integral_gain_;

    const ucb_jaco_control::Trajectory<StateDim>& desired_trajectory_;
    bool                                          wrap_angles_;

    Eigen::Matrix<double, StateDim, 1>            integral_;

    double                                        previous_time_;
};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_PID_TRAJECTORY_CONTROLLER_H
