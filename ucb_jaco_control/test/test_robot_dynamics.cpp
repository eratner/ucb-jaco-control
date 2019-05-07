#include <ucb_jaco_control/robot_dynamics.h>
#include <ucb_jaco_control/urdf_robot_dynamics.h>
#include <ucb_jaco_control/constant_trajectory.h>
#include <ucb_jaco_control/sinusoidal_trajectory.h>
#include <ucb_jaco_control/runge_kutta.h>
#include <ucb_jaco_control/pid_computed_torque_controller.h>
#include <gtest/gtest.h>
#include <iostream>
#include <exception>

#include <extern/matplotlibcpp/matplotlibcpp.h>

namespace plt = matplotlibcpp;

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
  const unsigned int N = 7;
  typedef Eigen::Matrix<double, 2*N, 1> StateVector;
  typedef Eigen::Matrix<double, N, 1>   ControlVector;

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

  std::function<StateVector (double, const StateVector&, const ControlVector&)> f =
    [&] (double t, const StateVector& state, const ControlVector& control)
  {
    Eigen::Matrix<double, N, 1> pos = state.block(0, 0, N, 1);
    Eigen::Matrix<double, N, 1> vel = state.block(N, 0, N, 1);

    Eigen::Matrix<double, N, N> M_inv = dynamics->getInertiaMatrix(pos).inverse();

    Eigen::Matrix<double, N, 1> acc = M_inv * (control -
                                               dynamics->getCoriolisVector(pos, vel) -
                                               dynamics->getGravityVector(pos) -
                                               dynamics->getFrictionVector(vel));

    StateVector state_deriv;
    state_deriv.block(0, 0, N, 1) = vel;
    state_deriv.block(N, 0, N, 1) = acc;
    return state_deriv;
  };

  Eigen::Matrix<double, N, 1> setpoint;
  setpoint << 0, 0, 0, 0, 0, 0, 0;
  ucb_jaco_control::ConstantTrajectory<N>* desired_trajectory =
    new ucb_jaco_control::ConstantTrajectory<N>(setpoint);

  std::array<double, N> p_gain;
  p_gain.fill(100.0);

  std::array<double, N> i_gain;
  i_gain.fill(0.0);

  std::array<double, 7> d_gain;
  d_gain.fill(20.0);

  ucb_jaco_control::PIDComputedTorqueController<N> controller(p_gain,
                                                              i_gain,
                                                              d_gain,
                                                              dynamics,
                                                              desired_trajectory);

  std::function<ControlVector (double, StateVector)> control = [&] (double t, StateVector state)
  {
    return controller.getControl(state, t);
  };

  ucb_jaco_control::RungeKutta<2*N, N> integrator(f, 0.01);

  StateVector state;
  //  state << 0, 0, 0, 0, 0, 0, 0,
  //    0, 0, 0, 0, 0, 0, 0;
  state << 0.1, 0.2, 0.1, 0.1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0;

  std::vector<std::vector<double> > pos;
  std::vector<std::vector<double> > des_pos;
  std::vector<std::vector<double> > vel;
  std::vector<double> times;

  // const int T = 1000;
  const int T = 500;
  for (int k = 0; k < T; ++k)
  {
    double t = k * integrator.getTimestep();

    std::vector<double> p;
    std::vector<double> dp;
    std::vector<double> v;
    for (int i = 0; i < N; ++i)
    {
      p.push_back(state(i));
      dp.push_back(desired_trajectory->getPosition(t)(i));
      v.push_back(state(N + i));
    }
    pos.push_back(p);
    des_pos.push_back(dp);
    vel.push_back(v);
    times.push_back(k * integrator.getTimestep());

    state = integrator.solve(state, control, t);
  }

   for (int i = 0; i < N; ++i)
   {
     std::vector<double> p;
     for (auto &ps : pos)
       p.push_back(ps[i]);

     std::vector<double> dp;
     for (auto &dps : des_pos)
       dp.push_back(dps[i]);

     plt::named_plot("J" + std::to_string(i) + " pos", times, p);
     plt::named_plot("J" + std::to_string(i) + " pos (des)", times, dp, "--");
   }

  //  plt::named_plot("J1 pos", times, pos[0], "r");
  //  plt::named_plot("J2 pos (des)", times, des_pos[0], "b--");

  plt::legend();
  plt::show();

  delete dynamics;
  dynamics = nullptr;

  delete desired_trajectory;
  desired_trajectory = nullptr;
}

TEST(RobotDynamics, controlledRobotDynamicsSinusoid)
{
  const unsigned int N = 7;
  typedef Eigen::Matrix<double, 2*N, 1> StateVector;
  typedef Eigen::Matrix<double, N, 1>   ControlVector;

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

  std::function<StateVector (double, const StateVector&, const ControlVector&)> f =
    [&] (double t, const StateVector& state, const ControlVector& control)
  {
    Eigen::Matrix<double, N, 1> pos = state.block(0, 0, N, 1);
    Eigen::Matrix<double, N, 1> vel = state.block(N, 0, N, 1);

    Eigen::Matrix<double, N, N> M_inv = dynamics->getInertiaMatrix(pos).inverse();

    Eigen::Matrix<double, N, 1> acc = M_inv * (control -
                                               dynamics->getCoriolisVector(pos, vel) -
                                               dynamics->getGravityVector(pos) -
                                               dynamics->getFrictionVector(vel));

    StateVector state_deriv;
    state_deriv.block(0, 0, N, 1) = vel;
    state_deriv.block(N, 0, N, 1) = acc;
    return state_deriv;
  };

  ucb_jaco_control::SinusoidalTrajectory<N>* desired_trajectory =
    new ucb_jaco_control::SinusoidalTrajectory<N>({0., 0.2, 0., 0., 0., 0., 0.},
      {1., 1., 1., 1., 1., 1., 1.},
      {0., 0., 0., 0., 0., 0., 0.});

  std::array<double, N> p_gain;
  p_gain.fill(100.0);

  std::array<double, N> i_gain;
  i_gain.fill(0.0);

  std::array<double, 7> d_gain;
  d_gain.fill(20.0);

  ucb_jaco_control::PIDComputedTorqueController<N> controller(p_gain,
                                                              i_gain,
                                                              d_gain,
                                                              dynamics,
                                                              desired_trajectory);

  std::function<ControlVector (double, StateVector)> control = [&] (double t, StateVector state)
  {
    return controller.getControl(state, t);
  };

  ucb_jaco_control::RungeKutta<2*N, N> integrator(f, 0.01);

  StateVector state;
  //  state << 0, 0, 0, 0, 0, 0, 0,
  //    0, 0, 0, 0, 0, 0, 0;
  state << 0.1, 0.2, 0.1, 0.1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0;

  std::vector<std::vector<double> > pos;
  std::vector<std::vector<double> > des_pos;
  std::vector<std::vector<double> > vel;
  std::vector<double> times;

  const int T = 1000;
  for (int k = 0; k < T; ++k)
  {
    double t = k * integrator.getTimestep();

    std::vector<double> p;
    std::vector<double> dp;
    std::vector<double> v;
    for (int i = 0; i < N; ++i)
    {
      p.push_back(state(i));
      dp.push_back(desired_trajectory->getPosition(t)(i));
      v.push_back(state(N + i));
    }
    pos.push_back(p);
    des_pos.push_back(dp);
    vel.push_back(v);
    times.push_back(k * integrator.getTimestep());

    state = integrator.solve(state, control, t);
  }

   for (int i = 0; i < N; ++i)
   {
     std::vector<double> p;
     for (auto &ps : pos)
       p.push_back(ps[i]);

     std::vector<double> dp;
     for (auto &dps : des_pos)
       dp.push_back(dps[i]);

     plt::named_plot("J" + std::to_string(i) + " pos", times, p);
     plt::named_plot("J" + std::to_string(i) + " pos (des)", times, dp, "--");
   }

  //  plt::named_plot("J1 pos", times, pos[0], "r");
  //  plt::named_plot("J2 pos (des)", times, des_pos[0], "b--");

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
