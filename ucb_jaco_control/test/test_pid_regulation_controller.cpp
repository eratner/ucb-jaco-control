#include <ucb_jaco_control/pid_regulation_controller.h>
#include <gtest/gtest.h>
#include <iostream>

TEST(PIDRegulationController, testConstruction)
{
  const unsigned int N = 3;
  ucb_jaco_control::PIDRegulationController<N> controller({1, 2, 3},
                                                          {10, 20, 30},
                                                          {100, 200, 300});

  ASSERT_FLOAT_EQ(1, controller.getProportionalGainMatrix().diagonal()(0));
  ASSERT_FLOAT_EQ(2, controller.getProportionalGainMatrix().diagonal()(1));
  ASSERT_FLOAT_EQ(3, controller.getProportionalGainMatrix().diagonal()(2));

  ASSERT_FLOAT_EQ(10, controller.getDerivativeGainMatrix().diagonal()(0));
  ASSERT_FLOAT_EQ(20, controller.getDerivativeGainMatrix().diagonal()(1));
  ASSERT_FLOAT_EQ(30, controller.getDerivativeGainMatrix().diagonal()(2));

  ASSERT_FLOAT_EQ(100, controller.getIntegralGainMatrix().diagonal()(0));
  ASSERT_FLOAT_EQ(200, controller.getIntegralGainMatrix().diagonal()(1));
  ASSERT_FLOAT_EQ(300, controller.getIntegralGainMatrix().diagonal()(2));
}

TEST(PIDRegulationController, testSingleIntegrator)
{
  // Simulation of a single integrator system (unit point mass in 1D).
  const unsigned int N = 1;
  const double       dt = 0.1;
  const unsigned int T = 25;

  ucb_jaco_control::PIDRegulationController<N>::StateVector setpoint;
  setpoint << 0;

  ucb_jaco_control::PIDRegulationController<N>::StateVector state;
  state << 1;

  ucb_jaco_control::PIDRegulationController<N> controller({1.75}, {0.25}, {0.0}, setpoint);

  for (int t = 0; t < T; ++t)
  {
    std::cout << "t = " << static_cast<double>(t) * dt << ", state = " << state << std::endl;

    ucb_jaco_control::PIDRegulationController<N>::ControlVector control =
      controller.getControl(state, dt);

    // std::cout << "  error = " << controller.getError() << std::endl;
    // std::cout << "  control = " << control << std::endl;

    // Update the state.
    state += dt * control;
  }
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
