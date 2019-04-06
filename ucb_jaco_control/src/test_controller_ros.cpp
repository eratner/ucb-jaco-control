#include <ucb_jaco_control/test_controller_ros.h>
#include <pluginlib/class_list_macros.hpp>

namespace ucb_jaco_control
{

TestControllerROS::TestControllerROS()
{
}

TestControllerROS::~TestControllerROS()
{
}

bool TestControllerROS::init(hardware_interface::EffortJointInterface* hw,
                             ros::NodeHandle& nh)
{
  ROS_INFO_STREAM("Initializing test controller");

  std::vector<std::string> joint_names;
  if (!nh.getParam("joints", joint_names))
  {
    ROS_ERROR("Failed to get joint names from parameter server!")
    return false;
  }

  for (int i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO_STREAM("Getting handle for joint " << joint_names[i]);
    joint_state_.push_back(hw->getHandle(joint_names[i]));
  }

  return true;
}

void TestControllerROS::starting(ros::Time &time)
{
  ROS_INFO_STREAM("Starting test controller at time " << time);
}

void TestControllerROS::update(const ros::Time& time, const ros::Duration& period)
{
}

void TestControllerROS::stopping(const ros::Time& time)
{
  ROS_INFO_STREAM("Stopping test controller at time " << time);
}

} // namespace ucb_jaco_control

PLUGINLIB_EXPORT_CLASS(ucb_jaco_control::TestControllerROS,
                       controller_interface::ControllerBase)
