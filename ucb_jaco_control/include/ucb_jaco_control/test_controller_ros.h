#ifndef UCB_JACO_CONTROL_TEST_CONTROLLER_ROS_H
#define UCB_JACO_CONTROL_TEST_CONTROLLER_ROS_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ucb_jaco_control/pid_regulation_controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <ucb_jaco_control/PIDGainsConfig.h>
#include <cmath>

#define P_GAIN 50.0
#define D_GAIN 0.0
#define I_GAIN 0.0

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
  void pidGainsCallback(PIDGainsConfig& config, uint32_t level);

  std::vector<hardware_interface::JointHandle> joint_handle_;

  PIDRegulationController<7>                   controller_;

  ros::Publisher                               error_pub_;

  dynamic_reconfigure::Server<PIDGainsConfig>* server_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_TEST_CONTROLLER_ROS_H
