#ifndef UCB_JACO_CONTROL_TEST_CONTROLLER_ROS_H
#define UCB_JACO_CONTROL_TEST_CONTROLLER_ROS_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ucb_jaco_control/pid_regulation_controller.h>

namespace ucb_jaco_control
{

class TestControllerROS :
    public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  TestControllerROS();

  ~TestControllerROS();

  bool init(hardware_interface::EffortJointInterface* hw,
            ros::NodeHandle& nh);

  void starting(ros::Time& time);

  void update(const ros::Time& time, const ros::Duration& period);

  void stopping(const ros::Time& time);

private:
  std::vector<hardware_interface::JointHandle> joint_handle_;

  PIDRegulationController<7>                   controller_;

  ros::Publisher                               error_pub_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_TEST_CONTROLLER_ROS_H
