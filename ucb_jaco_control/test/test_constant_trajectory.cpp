#include <ucb_jaco_control/trajectory.h>
#include <gtest/gtest.h>
#include <iostream>

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

  Eigen::Matrix<double, Dim, 1> getDesiredPosition(double t) const
  {
    return setpoint_;
  }

  Eigen::Matrix<double, Dim, 1> getDesiredVelocity(double t) const
  {
    return Eigen::Matrix<double, Dim, 1>::Zero();
  }

  Eigen::Matrix<double, Dim, 1> getDesiredAcceleration(double t) const
  {
    return Eigen::Matrix<double, Dim, 1>::Zero();
  }

private:
  Eigen::Matrix<double, Dim, 1> setpoint_;

};

}

TEST(ConstantTrajectory, testTraj)
{
  Eigen::Matrix<double, 2, 1> setpoint;
  setpoint << 1.5, 2.3;

  ucb_jaco_control::ConstantTrajectory<2> traj(setpoint);
  std::cout << traj.getDesiredPosition(0) << std::endl;
  std::cout << traj.getDesiredVelocity(0) << std::endl;
  std::cout << traj.getDesiredAcceleration(0) << std::endl;
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

