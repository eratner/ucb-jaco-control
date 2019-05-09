#ifndef UCB_JACO_CONTROL_PID_TRACKING_CONTROLLER_H
#define UCB_JACO_CONTROL_PID_TRACKING_CONTROLLER_H

#include <ucb_jaco_control/state_feedback_controller.h>
#include <ucb_jaco_control/trajectory.h>
#include <array>
#include <angles/angles.h>

namespace ucb_jaco_control
{

template <unsigned int Dim>
class PIDTrackingController : public StateFeedbackController<2*Dim, Dim>
{
public:
  using typename StateFeedbackController<2*Dim, Dim>::StateVector;
  using typename StateFeedbackController<2*Dim, Dim>::ControlVector;

  PIDTrackingController(const std::array<double, Dim>& proportional_gain,
                        const std::array<double, Dim>& integral_gain,
                        const std::array<double, Dim>& derivative_gain,
                        Trajectory<Dim>*               desired_trajectory,
                        bool                           wrap_angles = false)
    : desired_trajectory_(desired_trajectory), wrap_angles_(wrap_angles)
  {
    setProportionalGain(proportional_gain);
    setIntegralGain(integral_gain);
    setDerivativeGain(derivative_gain);
    reset();
  }

  ~PIDTrackingController()
  {
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

  void setIntegralGain(const std::array<double, Dim>& gain)
  {
    Eigen::Matrix<double, Dim, 1>& diag = integral_gain_.diagonal();
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

  const Eigen::DiagonalMatrix<double, Dim>& getIntegralGainMatrix() const
  {
    return integral_gain_;
  }

  void reset()
  {
    // Set integral to zero.
    integral_ = Eigen::Matrix<double, Dim, 1>::Zero();

    // Setting the time to < 0 will cause it to reset the next time a control is requested.
    last_time_ = -1.0;
  }

  ControlVector getControl(const StateVector& state, double t)
  {
    Eigen::Matrix<double, Dim, 1> curr_pos = state.block(0, 0, Dim, 1);
    Eigen::Matrix<double, Dim, 1> curr_vel = state.block(Dim, 0, Dim, 1);

    // get next point on desired trajectory
    Eigen::Matrix<double, Dim, 1> des_pos = desired_trajectory_->getPosition(t);
    Eigen::Matrix<double, Dim, 1> des_vel = desired_trajectory_->getVelocity(t);

    if (last_time_ < 0)
      last_time_ = t;

    double dt = t - last_time_;

    // Compute the error.
    Eigen::Matrix<double, Dim, 1> error = diff(des_pos, curr_pos);

    // Compute error derivative.
    Eigen::Matrix<double, Dim, 1> error_der = diff(des_vel, curr_vel);

    // Update the integral.
    integral_ += dt * error;

    return -1.0 * proportional_gain_ * error -
      integral_gain_ * integral_ -
      derivative_gain_ * error_der;
  }

protected:
    Eigen::Matrix<double, Dim, 1> diff(const Eigen::Matrix<double, Dim, 1>& a,
                                       const Eigen::Matrix<double, Dim, 1>& b) const
    {
      Eigen::Matrix<double, Dim, 1> d;
      if (wrap_angles_)
      {
        for (int i = 0; i < Dim; ++i)
          d(i) = angles::shortest_angular_distance(a(i), b(i));
      }
      else
        d = b - a;

      return d;
    }

    Eigen::DiagonalMatrix<double, Dim> proportional_gain_;
    Eigen::DiagonalMatrix<double, Dim> derivative_gain_;
    Eigen::DiagonalMatrix<double, Dim> integral_gain_;

    Trajectory<Dim>*                   desired_trajectory_;
    bool                               wrap_angles_;

    Eigen::Matrix<double, Dim, 1>      integral_;

    double                             last_time_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_PID_TRACKING_CONTROLLER_H
