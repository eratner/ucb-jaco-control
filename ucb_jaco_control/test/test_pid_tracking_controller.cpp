#include <ucb_jaco_control/pid_tracking_controller.h>
#include <ucb_jaco_control/sinusoidal_trajectory.h>
#include <extern/matplotlibcpp/matplotlibcpp.h>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>

namespace plt = matplotlibcpp;

TEST(PIDTrajectoryController, testSingleIntegrator)
{
  const unsigned int N = 1;
  const double       dt = 0.1;
  const unsigned int T = 250;

  ucb_jaco_control::SinusoidalTrajectory<N>* trajectory =
    new ucb_jaco_control::SinusoidalTrajectory<N>({1}, {1}, {0}, {0});

  ucb_jaco_control::PIDTrackingController<N>::StateVector state;
  state << sin(1), 0;

  ucb_jaco_control::PIDTrackingController<N> controller({2}, {1}, {0.4}, trajectory);

  std::vector<double> pos;
  std::vector<double> times;
  std::vector<double> setpoints;

  for (int t = 1; t <= T; ++t)
  {
    pos.push_back(state(0));
    times.push_back(t * dt);
    setpoints.push_back(trajectory->getPosition(t * dt)(0));

    std::cout << "t = " << static_cast<double>(t) * dt << ", state: " << state <<
      ", des: " << trajectory->getPosition(t) << std::endl;

    ucb_jaco_control::PIDTrackingController<N>::ControlVector control =
      controller.getControl(state, t * dt);

    state(0) += dt * control(0);
    state(1) = control(0);
  }

  delete trajectory;
  trajectory = nullptr;

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
