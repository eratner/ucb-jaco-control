#ifndef UCB_JACO_CONTROL_SINUSOIDAL_TRAJECTORY_H
#define UCB_JACO_CONTROL_SINUSOIDAL_TRAJECTORY_H

#include <ucb_jaco_control/trajectory.h>
#include <cmath>

namespace ucb_jaco_control
{

template <unsigned int Dim>
class SinusoidalTrajectory : public Trajectory<Dim>
{
public:
  SinusoidalTrajectory(const std::array<double, Dim>& amplitude,
                       const std::array<double, Dim>& frequency,
                       const std::array<double, Dim>& phase,
                       const std::array<double, Dim>& offset)
    : amplitude_(amplitude), frequency_(frequency), phase_(phase), offset_(offset)
  {
  }

  Eigen::Matrix<double, Dim, 1> getPosition(double t)
  {
    Eigen::Matrix<double, Dim, 1> pos = Eigen::Matrix<double, Dim, 1>::Zero();
    for (int i = 0; i < Dim; i++)
      pos(i) = offset_[i] + amplitude_[i] * sin(frequency_[i] * t + phase_[i]);

    return pos;
  }

  Eigen::Matrix<double, Dim, 1> getVelocity(double t)
  {
    Eigen::Matrix<double, Dim, 1> vel = Eigen::Matrix<double, Dim, 1>::Zero();
    for (int i = 0; i < Dim; i++)
    {
      vel(i) = amplitude_[i] * frequency_[i] *
        cos(frequency_[i] * t + phase_[i]);
    }

    return vel;
  }

  Eigen::Matrix<double, Dim, 1> getAcceleration(double t)
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
  std::array<double, Dim> offset_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_SINUSOIDAL_TRAJECTORY_H
