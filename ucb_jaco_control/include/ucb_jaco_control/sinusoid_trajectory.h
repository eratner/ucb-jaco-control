#ifndef UCB_JACO_CONTROL_SINUSOID_TRAJECTORY_H
#define UCB_JACO_CONTROL_SINUSOID_TRAJECTORY_H

#include <ucb_jaco_control/trajectory.h>
#include <cmath>

namespace ucb_jaco_control
{

template <unsigned int Dim>
class SinusoidTrajectory : public Trajectory<Dim>
{
public:
  SinusoidTrajectory(const std::array<double, Dim>& amplitude,
                     const std::array<double, Dim>& frequency,
                     const std::array<double, Dim>& phase)
    : amplitude_(amplitude), frequency_(frequency), phase_(phase)
  {
  }

  Eigen::Matrix<double, Dim, 1> getPosition(double t) const
  {
    Eigen::Matrix<double, Dim, 1> pos = Eigen::Matrix<double, StateDim, 1>::Zero();
    for (int i = 0; i < Dim; i++)
      pos(i) = amplitude_[i] * sin(frequency_[i] * t + phase_[i]);
    
    return pos;
  }

  Eigen::Matrix<double, Dim, 1> getVelocity(double t) const
  {
    Eigen::Matrix<double, Dim, 1> vel = Eigen::Matrix<double, Dim, 1>::Zero();
    for (int i = 0; i < Dim; i++)
    {
      vel(i) = amplitude_[i] * frequency_[i] *
        cos(frequency_[i] * t + phase_[i]);
    }

    return vel;
  }

  Eigen::Matrix<double, Dim, 1> getAcceleration(double t) const
  {
    Eigen::Matrix<double, Dim, 1> acc = Eigen::Matrix<double, Dim, 1>::Zero();
    for (int i = 0; i < Dim; i++)
    {
      acc(i) = -amplitude_[i] * pow(frequency_[i], 2) *
        sin(frequency_[i] * t + phase_[i]);
    }

    return acc;
  }

protected:
  std::array<double, Dim> amplitude_;
  std::array<double, Dim> frequency_;
  std::array<double, Dim> phase_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_SINUSOID_TRAJECTORY_H
