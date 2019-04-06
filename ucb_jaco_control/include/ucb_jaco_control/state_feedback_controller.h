#ifndef UCB_JACO_CONTROL_STATE_FEEDBACK_CONTROLLER_H
#define UCB_JACO_CONTROL_STATE_FEEDBACK_CONTROLLER_H

#include <Eigen/Eigen>

namespace ucb_jaco_control
{

template <unsigned int StateDim, unsigned int ControlDim>
class StateFeedbackController
{
public:
  typedef Eigen::Matrix<double, StateDim, 1>   StateVector;
  typedef Eigen::Matrix<double, ControlDim, 1> ControlVector;

  virtual ControlVector getControl(const StateVector& state, double dt) = 0;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_STATE_FEEDBACK_CONTROLLER_H
