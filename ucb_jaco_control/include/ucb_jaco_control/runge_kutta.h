#ifndef UCB_JACO_CONTROL_RUNGE_KUTTA_H_
#define UCB_JACO_CONTROL_RUNGE_KUTTA_H_

#include <Eigen/Eigen>
#include <functional>
#include <iostream>

namespace ucb_jaco_control
{

template <unsigned int StateDim, unsigned int ControlDim>
class RungeKutta
{
public:
  typedef Eigen::Matrix<double, StateDim, 1>   StateVector;
  typedef Eigen::Matrix<double, ControlDim, 1> ControlVector;

  // Constructs a Runge-Kutta method to solve the ODE \dot{x} = f(t, x, u),
  // for some initial condition to be specified later. Here, t is a nonnegative
  // real-valued time, x is a StateDim-dimensional state vector, and u is a
  // ControlDim-dimensional control vector. Nonnegative integration step size is h.
  RungeKutta(std::function<StateVector (double, StateVector, ControlVector)> f, double h)
      : f_(f), h_(h)
  {
  }

  // Apply the Runge-Kutta method (RK4) to solve the ODE for the given
  // initial condition x(t0) = x0 and control function u, for a single
  // time step.
  StateVector solve(const StateVector& x0,
                    std::function<ControlVector (double, StateVector)> u
                    = [&](double t, const StateVector &x) -> ControlVector { return ControlVector::Zero(); },
                     double t0 = 0) const
  {
    StateVector x;

    StateVector k1, k2, k3, k4;
    ControlVector u0 = u(t0, x0);

    k1 = h_ * f_(t0, x0, u0);
    k2 = h_ * f_(t0 + 0.5 * h_, x0 + 0.5 * k1, u0);
    k3 = h_ * f_(t0 + 0.5 * h_, x0 + 0.5 * k2, u0);
    k4 = h_ * f_(t0 + h_, x0 + k3, u0);

    x = x0 + (k1 + 2 * k2 + 2 * k3 + k4) / 6;

    return x;
  }

  double getTimestep() const
  {
    return h_;
  }

  void setTimestep(double h)
  {
    h_ = h;
  }

private:
  std::function<StateVector (double, StateVector, ControlVector)> f_;
  double                                                          h_;

};

}  // namespace ucb_jaco_control

#endif  // UCB_JACO_CONTROL_RUNGE_KUTTA_H_
