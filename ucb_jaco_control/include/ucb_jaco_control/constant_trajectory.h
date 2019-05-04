#ifndef UCB_JACO_CONTROL_CONSTANT_TRAJECTORY_H
#define UCB_JACO_CONTROL_CONSTANT_TRAJECTORY_H

#include <ucb_jaco_control/trajectory.h>

namespace ucb_jaco_control
{

template <unsigned int Dim>
class ConstantTrajectory : public Trajectory<Dim>
{
public:
  ConstantTrajectory(const Eigen::Matrix<double, Dim, 1> &setpoint)
    : setpoint_(setpoint)
  {
  }

  Eigen::Matrix<double, Dim, 1> getPosition(double t)
  {
    return setpoint_;
  }

  Eigen::Matrix<double, Dim, 1> getVelocity(double t)
  {
    return Eigen::Matrix<double, Dim, 1>::Zero();
  }

  Eigen::Matrix<double, Dim, 1> getAcceleration(double t)
  {
    return Eigen::Matrix<double, Dim, 1>::Zero();
  }

private:
  Eigen::Matrix<double, Dim, 1> setpoint_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_CONSTANT_TRAJECTORY_H
