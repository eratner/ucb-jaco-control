#ifndef UCB_JACO_CONTROL_ROBOT_DYNAMICS_H
#define UCB_JACO_CONTROL_ROBOT_DYNAMICS_H

#include <Eigen/Eigen>

namespace ucb_jaco_control
{

template <unsigned int DOF>
class RobotDynamics
{
public:
  typedef Eigen::Matrix<double, DOF, DOF> Matrix;
  typedef Eigen::Matrix<double, DOF, 1>   Vector;

  virtual Matrix getInertiaMatrix(const Vector& pos) = 0;
  virtual Vector getCoriolisVector(const Vector& pos,
                                   const Vector& vel) = 0;

  virtual Vector getGravityVector(const Vector &pos) = 0;

  virtual Vector getFrictionVector(const Vector &vel) = 0;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_ROBOT_DYNAMICS_H
