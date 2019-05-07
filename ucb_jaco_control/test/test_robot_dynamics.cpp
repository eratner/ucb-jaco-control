#include <ucb_jaco_control/robot_dynamics.h>
#include <ucb_jaco_control/urdf_robot_dynamics.h>
#include <gtest/gtest.h>
#include <iostream>
#include <exception>

TEST(RobotDynamics, testLoadURDF)
{
  // Generated URDF using command
  // rosrun xacro xacro.py `rospack find
  //   kinova_description`/urdf/j2s7s300_standalone.xacro > j2s7s300.urdf
  std::string urdf_file = "/home/eratner/j2s7s300.urdf";
  std::string root_name = "j2s7s300_link_base";
  std::string tip_name  = "j2s7s300_end_effector";

  ucb_jaco_control::RobotDynamics<7> *dynamics = nullptr;

  try
  {
    dynamics = new ucb_jaco_control::URDFRobotDynamics<7>(urdf_file, root_name, tip_name);
  }
  catch (const std::exception &e)
  {
    std::cout << "Error: " << e.what() << std::endl;
  }

  Eigen::Matrix<double, 7, 1> pos;
  pos << 0, M_PI_2, 0, 0, 0, 0, 0;
  Eigen::Matrix<double, 7, 7> M = dynamics->getInertiaMatrix(pos);
  std::cout << "%% Inertia Matrix: " << std::endl;
  std::cout << M << std::endl;

  Eigen::Matrix<double, 7, 1> vel;
  vel << 0, 0, 0, 0, 0, 0, 0;
  Eigen::Matrix<double, 7, 1> C = dynamics->getCoriolisVector(pos, vel);
  std::cout << "%% Coriolis Vector: " << std::endl;
  std::cout << C << std::endl;

  Eigen::Matrix<double, 7, 1> G = dynamics->getGravityVector(pos);
  std::cout << "%% Gravity Vector: " << std::endl;
  std::cout << G << std::endl;

  delete dynamics;
  dynamics = nullptr;
}

TEST(RobotDynamics, controlledRobotDynamics)
{
  // TODO: Write test with URDF dynamics + integrator + PID CTC
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
