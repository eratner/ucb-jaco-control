#ifndef UCB_JACO_CONTROL_TWO_LINK_PLANAR_ARM_DYNAMICS_H
#define UCB_JACO_CONTROL_TWO_LINK_PLANAR_ARM_DYNAMICS_H

#include <ucb_jaco_control/robot_dynamics.h>
#include <cmath>

namespace ucb_jaco_control
{

class TwoLinkPlanarArmDynamics : public RobotDynamics<2>
{
public:
  typedef Eigen::Matrix<double, 2, 2> Matrix;
  typedef Eigen::Matrix<double, 2, 1> Vector;

  TwoLinkPlanarArmDynamics(double l1 = 0.2,
                           double l2 = 0.2,
                           double m1 = 1.0,
                           double m2 = 1.0,
                           double g  = 9.81)
    : l1_(l1), l2_(l2), m1_(m1), m2_(m2), g_(g)
  {
  }

  ~TwoLinkPlanarArmDynamics()
  {
  }

  Matrix getInertiaMatrix(const Vector& pos)
  {
    const double &q2 = pos(1);
    Matrix inertia_matrix;
    inertia_matrix(0, 0) =
      (m1_ + m2_) * pow(l1_, 2) + m2_ * pow(l2_, 2) + 2 * m2_ * l1_ * l2_ * cos(q2);
    inertia_matrix(0, 1) = m2_ * pow(l2_, 2) + m2_ * l1_ * l2_ * cos(q2);
    inertia_matrix(1, 0) = m2_ * pow(l2_, 2) + m2_ * l1_ * l2_ * cos(q2);
    inertia_matrix(1, 1) = m2_ * pow(l2_, 2);

    return inertia_matrix;
  }

  Vector getCoriolisVector(const Vector& pos,
                           const Vector& vel)
  {
    const double &q1 = pos(0);
    const double &q2 = pos(1);
    const double &dq1 = vel(0);
    const double &dq2 = vel(1);

    Vector coriolis_centripetal;
    coriolis_centripetal(0) = -m2_ * l1_ * l2_ * (2 * dq1 * dq2 + pow(dq2, 2)) * sin(dq2);
    coriolis_centripetal(1) = m2_ * l1_ * l2_ * pow(dq1, 2) * sin(q2);

    return coriolis_centripetal;
  }

  Vector getGravityVector(const Vector& pos)
  {
    const float &q1 = pos(0, 0);
    const float &q2 = pos(1, 0);

    Vector gravity;
    gravity(0) = (m1_ + m2_) * g_ * l1_ * cos(q1) + m2_ * g_ * l2_ * cos(q1 + q2);
    gravity(1) = m2_ * g_ * l2_ * cos(q1 + q2);

    return gravity;
  }

  Vector getFrictionVector(const Vector& vel)
  {
    // TODO: For now, assume zero friction.
    return Vector::Zero();
  }

private:
  double l1_; // Length of the first link (m).
  double l2_; // Length of the second link (m).
  double m1_; // Mass of the first link (kg).
  double m2_; // Mass of the second link (kg).
  double g_;  // Acceleration due to gravity (m/s/s).

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_TWO_LINK_PLANAR_ARM_DYNAMICS_H
