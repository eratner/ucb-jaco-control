#include <ucb_jaco_control/trajectory.h>
#include <ucb_jaco_control/two_link_planar_arm_dynamics.h>
#include <ucb_jaco_control/pd_computed_torque_controller.h>
#include <ucb_jaco_control/runge_kutta.h>

#include <gtest/gtest.h>
#include <iostream>

#include <extern/matplotlibcpp/matplotlibcpp.h>

namespace plt = matplotlibcpp;

namespace ucb_jaco_control
{

class TestTraj1 : public Trajectory<2>
{
public:
  const float g1    = 0.1;
  const float g2    = 0.1;
  const float per   = 2.;
  const float twopi = 6.283;

  Eigen::Matrix<double, 2, 1> getPosition(double t)
  {
    const float fact = twopi / per;
    Eigen::Matrix<double, 2, 1> pos;
    pos << g1 * sin(fact * t), g2 * cos(fact * t);
    return pos;
  }

  Eigen::Matrix<double, 2, 1> getVelocity(double t)
  {
    const float fact = twopi / per;
    Eigen::Matrix<double, 2, 1> vel;
    vel << g1 * fact * cos(fact * t), -g2 * fact * sin(fact * t);
    return vel;
  }

  Eigen::Matrix<double, 2, 1> getAcceleration(double t)
  {
    const float fact = twopi / per;
    Eigen::Matrix<double, 2, 1> acc;
    acc << -g1 * pow(fact, 2) * sin(fact * t), -g2 * pow(fact, 2) * cos(fact * t);
    return acc;
  }

};

} // namespace ucb_jaco_control

TEST(PDComputedTorqueControl, testControl)
{
  typedef Eigen::Matrix<double, 4, 1> StateVector;
  typedef Eigen::Matrix<double, 2, 1> ControlVector;

  ucb_jaco_control::TwoLinkPlanarArmDynamics* dynamics =
    new ucb_jaco_control::TwoLinkPlanarArmDynamics(1., 1., 1., 1.);

  std::function<StateVector (double, const StateVector&, const ControlVector&)> f =
    [&] (double t, const StateVector& state, const ControlVector& control)
  {
    Eigen::Matrix<double, 2, 1> pos = state.block(0, 0, 2, 1);
    Eigen::Matrix<double, 2, 1> vel = state.block(2, 0, 2, 1);

    Eigen::Matrix<double, 2, 2> M_inv = dynamics->getInertiaMatrix(pos).inverse();

    // Assuming no control input.
    Eigen::Matrix<double, 2, 1> acc = M_inv * (control - dynamics->getCoriolisVector(pos, vel) -
                                               dynamics->getGravityVector(pos) -
                                               dynamics->getFrictionVector(vel));

    StateVector state_deriv;
    state_deriv.block(0, 0, 2, 1) = vel;
    state_deriv.block(2, 0, 2, 1) = acc;
    return state_deriv;
  };

  ucb_jaco_control::TestTraj1* desired_trajectory =
    new ucb_jaco_control::TestTraj1();

  ucb_jaco_control::PDComputedTorqueController<2> controller({100., 100.},
                                                             {20. , 20. },
                                                             dynamics,
                                                             desired_trajectory);

  std::function<ControlVector (double, StateVector)> control = [&] (double t, StateVector state)
  {
    return controller.getControl(state, t);
  };

  ucb_jaco_control::RungeKutta<4, 2> integrator(f, 0.01);

  StateVector state;
  state << 0.0, 0.0, 0.0, 0.0;

  std::vector<double> pos1;
  std::vector<double> des_pos1;
  std::vector<double> pos2;
  std::vector<double> des_pos2;
  std::vector<double> vel1;
  std::vector<double> vel2;
  std::vector<double> times;

  const int T = 1000;
  for (int k = 0; k < T; ++k)
  {
    double t = k * integrator.getTimestep();

    pos1.push_back(state(0));
    des_pos1.push_back(desired_trajectory->getPosition(t)(0));
    pos2.push_back(state(1));
    des_pos2.push_back(desired_trajectory->getPosition(t)(1));
    vel1.push_back(state(2));
    vel2.push_back(state(3));
    times.push_back(k * integrator.getTimestep());

    state = integrator.solve(state, control, t);
  }

  plt::named_plot("Pos joint 1", times, pos1, "r");
  plt::named_plot("Pos joint 2", times, pos2, "b");
  plt::named_plot("Pos joint 1 desired", times, des_pos1, "r--");
  plt::named_plot("Pos joint 2 desired", times, des_pos2, "b--");
  // plt::named_plot("Vel joint 1", times, vel1);
  // plt::named_plot("Vel joint 2", times, vel2);
  plt::legend();
  plt::show();

  delete dynamics;
  dynamics = nullptr;

  delete desired_trajectory;
  desired_trajectory = nullptr;
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

