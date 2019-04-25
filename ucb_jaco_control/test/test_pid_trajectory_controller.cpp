#include <ucb_jaco_control/pid_trajectory_controller.h>
#include <ucb_jaco_control/sinusoid_trajectory.h>
#include <extern/matplotlibcpp/matplotlibcpp.h>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>

namespace plt = matplotlibcpp;

TEST(PIDTrajectoryController, testSingleIntegrator)
{
  const unsigned int          N = 1;
  const double                dt = 3.14159 / 8;
  const unsigned int          T = 100;

  ucb_jaco_control::SinusoidTrajectory<N> trajectory({1}, {1}, {0});

  ucb_jaco_control::PIDTrajectoryController<N>::StateVector state;
  state << sin(1), 0;

  ucb_jaco_control::PIDTrajectoryController<N> controller({2}, {1}, {0.4}, trajectory);

  std::vector<double> pos;
  std::vector<double> times;
  std::vector<double> setpoints;

  for (int t = 1; t <= T; ++t)
  {
    pos.push_back(state(0));
    times.push_back(t * dt);
    setpoints.push_back(trajectory.getDesiredPosition(t * dt)(0));

    std::cout << "t = " << static_cast<double>(t) * dt << ", state: " << state << 
      ", des: " << trajectory.getDesiredPosition(t) << std::endl;

    ucb_jaco_control::PIDTrajectoryController<N>::ControlVector control = 
      controller.getControl(state, t * dt);

    state(0) += dt * control(0);
    state(1) = control(0);
  }

  plt::named_plot("State", times, pos);
  plt::named_plot("Setpoint", times, setpoints);
  plt::legend();
  plt::show();
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
