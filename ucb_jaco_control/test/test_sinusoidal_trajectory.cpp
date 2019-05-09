#include <ucb_jaco_control/sinusoidal_trajectory.h>
#include <gtest/gtest.h>
#include <iostream>

#include <extern/matplotlibcpp/matplotlibcpp.h>

namespace plt = matplotlibcpp;

TEST(SinusoidalTrajectory, testMultiDimSinusoid)
{
  ucb_jaco_control::SinusoidalTrajectory<3> trajectory({1., 2., 2.},
                                                       {0., 1., 1.},
                                                       {0., 0., M_PI_2},
                                                       {0., 0., 0.});

  std::vector<double> sin1;
  std::vector<double> sin2;
  std::vector<double> sin3;
  std::vector<double> deriv_sin1;
  std::vector<double> deriv_sin2;
  std::vector<double> deriv_sin3;
  std::vector<double> times;

  const int T = 100;
  double dt = 0.1;
  for (int i = 0; i < T; ++i)
  {
    double t = i * dt;
    times.push_back(t);

    Eigen::Matrix<double, 3, 1> p = trajectory.getPosition(t);
    sin1.push_back(p(0));
    sin2.push_back(p(1));
    sin3.push_back(p(2));

    Eigen::Matrix<double, 3, 1> v = trajectory.getVelocity(t);
    deriv_sin1.push_back(v(0));
    deriv_sin2.push_back(v(1));
    deriv_sin3.push_back(v(2));
  }

  plt::figure(1);
  plt::named_plot("Pos 1", times, sin1);
  plt::named_plot("Pos 2", times, sin2);
  plt::named_plot("Pos 3", times, sin3);
  plt::legend();

  plt::figure(2);
  plt::named_plot("Vel 1", times, deriv_sin1);
  plt::named_plot("Vel 2", times, deriv_sin2);
  plt::named_plot("Vel 3", times, deriv_sin3);
  plt::legend();

  plt::show();
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
