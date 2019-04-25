#ifndef UCB_JACO_CONTROL_TRAJECTORY_H
#define UCB_JACO_CONTROL_TRAJECTORY_H

#include <Eigen/Eigen>

namespace ucb_jaco_control
{

template <unsigned int Dim>
class Trajectory
{
public:
  virtual Eigen::Matrix<double, Dim, 1> getDesiredPosition(double t) const = 0;

  virtual Eigen::Matrix<double, Dim, 1> getDesiredVelocity(double t) const = 0;

  virtual Eigen::Matrix<double, Dim, 1> getDesiredAcceleration(double t) const = 0;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_TRAJECTORY_H
