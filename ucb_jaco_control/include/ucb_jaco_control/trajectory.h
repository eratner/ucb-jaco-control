#ifndef UCB_JACO_CONTROL_TRAJECTORY_H
#define UCB_JACO_CONTROL_TRAJECTORY_H

#include <Eigen/Eigen>

namespace ucb_jaco_control
{

template <unsigned int Dim>
class Trajectory
{
public:
  virtual Eigen::Matrix<double, Dim, 1> getPosition(double t) = 0;

  virtual Eigen::Matrix<double, Dim, 1> getVelocity(double t) = 0;

  virtual Eigen::Matrix<double, Dim, 1> getAcceleration(double t) = 0;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_TRAJECTORY_H
