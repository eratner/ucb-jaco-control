#include <ucb_jaco_control/constant_trajectory.h>
#include <gtest/gtest.h>
#include <iostream>

TEST(ConstantTrajectory, testTraj)
{
  Eigen::Matrix<double, 2, 1> setpoint;
  setpoint << 1.5, 2.3;

  ucb_jaco_control::ConstantTrajectory<2> traj(setpoint);
  std::cout << traj.getPosition(0) << std::endl;
  std::cout << traj.getVelocity(0) << std::endl;
  std::cout << traj.getAcceleration(0) << std::endl;
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

