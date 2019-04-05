#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <string>

namespace ucb_jaco_control
{

class TorqueControlTestNode
{
public:
  TorqueControlTestNode(double loop_freq = 100.0)
    : loop_freq_(loop_freq), p_nh_("~")
  {
    // Query the parameter server for the topics.
    std::string joint_states_topic = p_nh_.param<std::string>("joint_states_topic",
                                                              "/j2s7s300/joint_states");
    std::string joint_torques_topic = p_nh_.param<std::string>("joint_torques_topic",
                                                               "/j2s7s300/effort_joint_trajectory_controller/command");

    // Set up the joint states subscriber and joint torques publisher.
    joint_states_sub_ = nh_.subscribe(joint_states_topic,
                                      10,
                                      &TorqueControlTestNode::jointStatesCallback,
                                      this);
    joint_torques_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(joint_torques_topic, 10);
  }

  ~TorqueControlTestNode()
  {
  }

  void run()
  {
    ros::Rate rate(loop_freq_);

    while (ros::ok())
    {
      // Compute and publish the commanded torques.
      trajectory_msgs::JointTrajectory control_msg = pdTorqueControl();
      joint_torques_pub_.publish(control_msg);

      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  trajectory_msgs::JointTrajectory pdTorqueControl()
  {
    trajectory_msgs::JointTrajectory control;
    control.header.stamp = ros::Time::now();

    control.joint_names = {
      "j2s7s300_joint_1",
      "j2s7s300_joint_2",
      "j2s7s300_joint_3",
      "j2s7s300_joint_4",
      "j2s7s300_joint_5",
      "j2s7s300_joint_6",
      "j2s7s300_joint_7",
      "j2s7s300_joint_finger_1",
      "j2s7s300_joint_finger_2",
      "j2s7s300_joint_finger_3"
    };

    //! TODO: actually compute some interesting control.
    trajectory_msgs::JointTrajectoryPoint point;
    point.effort = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    point.time_from_start = ros::Duration(0);

    //! TODO: compute control input.

    return control;
  }

  void jointStatesCallback(const sensor_msgs::JointState &msg)
  {
    current_joint_states_ = msg;
  }

  double          loop_freq_;         //! Frequency of the control loop (Hz).

  ros::NodeHandle nh_;                //! Node handle.
  ros::NodeHandle p_nh_;              //! Private node handle.

  ros::Subscriber joint_states_sub_;  //! Joint states subscriber.
  ros::Publisher  joint_torques_pub_; //! Joint torques publisher.

  sensor_msgs::JointState current_joint_states_;

};

} // namespace ucb_jaco_control

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "torque_control_test_node");

  ucb_jaco_control::TorqueControlTestNode test_node;

  test_node.run();

  return 0;
}
