#include <ucb_jaco_control/runge_kutta.h>
#include <ucb_jaco_control/two_link_planar_arm_dynamics.h>
#include <gtest/gtest.h>
#include <iostream>
#include <extern/matplotlibcpp/matplotlibcpp.h>

namespace plt = matplotlibcpp;

TEST(RungeKuttaTest, simpleTest)
{
  typedef Eigen::Matrix<double, 1, 1> StateVector;
  typedef Eigen::Matrix<double, 1, 1> ControlVector;

  const float h = 0.1;

  ucb_jaco_control::RungeKutta<1, 1> solver([] (double t, StateVector x, ControlVector u) -> StateVector {
      StateVector xdot;
      xdot << (5. * pow(t, 2) - x(0)) / (exp(t + x(0)));
      return xdot;
    }, h);

  StateVector x0;
  x0(0) = 1;

  float T = 2.0;
  StateVector x = x0;
  double t = 0.0;
  while (t < T)
  {
    x = solver.solve(x, [] (double t, StateVector x) { return ControlVector::Zero(); }, t);
    t += h;
    std::cout << "t = " << t << ", x = " << x << std::endl;
  }
}

TEST(RungeKuttaTest, twoLinkRobotTest)
{
  typedef Eigen::Matrix<double, 4, 1> StateVector;
  typedef Eigen::Matrix<double, 2, 1> ControlVector;

  ucb_jaco_control::TwoLinkPlanarArmDynamics dynamics(1., 1., 1., 1.);

  std::function<StateVector (double, const StateVector&, const ControlVector&)> f =
    [&] (double t, const StateVector& state, const ControlVector& control)
  {
    Eigen::Matrix<double, 2, 1> pos = state.block(0, 0, 2, 1);
    Eigen::Matrix<double, 2, 1> vel = state.block(2, 0, 2, 1);

    Eigen::Matrix<double, 2, 2> M_inv = dynamics.getInertiaMatrix(pos).inverse();

    // Assuming no control input.
    Eigen::Matrix<double, 2, 1> acc = M_inv * (-dynamics.getCoriolisVector(pos, vel) -
                                               dynamics.getGravityVector(pos) -
                                               dynamics.getFrictionVector(vel));

    StateVector state_deriv;
    state_deriv.block(0, 0, 2, 1) = vel;
    state_deriv.block(2, 0, 2, 1) = acc;
    return state_deriv;
  };

  std::function<ControlVector (double, StateVector)> u = [] (double t, StateVector x)
  {
    return ControlVector::Zero();
  };

  ucb_jaco_control::RungeKutta<4, 2> integrator(f, 0.1);

  StateVector x;
  x << 0.0, 0.0, 0.0, 0.0;

  std::vector<double> pos1;
  std::vector<double> pos2;
  std::vector<double> vel1;
  std::vector<double> vel2;
  std::vector<double> times;

  const int T = 100;
  for (int k = 0; k < T; ++k)
  {
    pos1.push_back(x(0));
    pos2.push_back(x(1));
    vel1.push_back(x(2));
    vel2.push_back(x(3));
    times.push_back(k * integrator.getTimestep());

    x = integrator.solve(x, u);
  }

  plt::named_plot("Pos joint 1", times, pos1);
  plt::named_plot("Pos joint 2", times, pos2);
  plt::named_plot("Vel joint 1", times, vel1);
  plt::named_plot("Vel joint 2", times, vel2);
  plt::legend();
  plt::show();
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

