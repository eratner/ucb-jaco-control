#ifndef UCB_JACO_CONTROL_SINUSOID_TRAJECTORY_H
#define UCB_JACO_CONTROL_SINUSOID_TRAJECTORY_H

#include <Eigen/Eigen>
#include <cmath>

namespace ucb_jaco_control
{

template <unsigned int StateDim>
class SinusoidTrajectory : public Trajectory<StateDim>
{
public:
  SinusoidTrajectory(const std::array<double, StateDim>& amplitude,
                     const std::array<double, StateDim>& frequency,
                     const std::array<double, StateDim>& phase)
    : amplitude_(amplitude), frequency_(frequency), phase_(phase)
  {  
  }

  Eigen::Matrix<double, StateDim, 1> getDesiredPosition(double t) const
  {
    Eigen::Matrix<double, StateDim, 1> pos = Eigen::Matrix<double, StateDim, 1>::Zero();
    for (int i = 0; i < StateDim; i++)
      pos(i) = amplitude_[i] * sin(frequency_[i] * t + phase_[i]);
    return pos;
  }

  Eigen::Matrix<double, StateDim, 1> getDesiredVelocity(double t) const
  {
    Eigen::Matrix<double, StateDim, 1> vel = Eigen::Matrix<double, StateDim, 1>::Zero();
    for (int i = 0; i < StateDim; i++)
      vel(i) = amplitude_[i] * frequency_[i] * 
               cos(frequency_[i] * t + phase_[i]);
    return vel;
  }

  Eigen::Matrix<double, StateDim, 1> getDesiredAcceleration(double t) const
  {
    Eigen::Matrix<double, StateDim, 1> acc = Eigen::Matrix<double, StateDim, 1>::Zero();
    for (int i = 0; i < StateDim; i++)
      acc(i) = amplitude_[i] * pow(frequency_[i], 2) * 
               -sin(frequency_[i] * t + phase_[i]);
    return acc;
  }

protected:
  std::array<double, StateDim>               amplitude_;
  std::array<double, StateDim>               frequency_;
  std::array<double, StateDim>               phase_;
};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_SINUSOID_TRAJECTORY_H
