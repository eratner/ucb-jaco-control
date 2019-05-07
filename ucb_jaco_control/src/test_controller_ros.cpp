#include <ucb_jaco_control/test_controller_ros.h>
#include <pluginlib/class_list_macros.hpp>

namespace ucb_jaco_control
{

TestControllerROS::TestControllerROS()
  : dynamics_(nullptr), desired_trajectory_(nullptr), controller_(nullptr)
{
  // controller_ = new PIDRegulationController<7>(
  //   {P_GAIN, P_GAIN, P_GAIN, P_GAIN, P_GAIN, P_GAIN, P_GAIN},
  //   {I_GAIN, I_GAIN, I_GAIN, I_GAIN, I_GAIN, I_GAIN, I_GAIN},
  //   {D_GAIN, D_GAIN, D_GAIN, D_GAIN, D_GAIN, D_GAIN, D_GAIN},
  //   PIDRegulationController<7>::StateVector::Zero(),
  //   true);
}

TestControllerROS::~TestControllerROS()
{
  delete controller_;
  controller_ = nullptr;

  delete dynamics_;
  dynamics_ = nullptr;

  delete desired_trajectory_;
  desired_trajectory_ = nullptr;

  delete server_;
  server_ = nullptr;
}

bool TestControllerROS::init(hardware_interface::EffortJointInterface* hw,
                             ros::NodeHandle& nh)
{
  ROS_INFO_STREAM("Initializing test controller");

  std::vector<std::string> joint_names;
  if (!nh.getParam("joints", joint_names))
  {
    ROS_ERROR("Failed to get joint names from parameter server!");
    return false;
  }

  for (int i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO_STREAM("Getting handle for joint " << joint_names[i]);
    try
    {
      joint_handle_.push_back(hw->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Error: " << e.what());
      return false;
    }
  }

  // HACK: Replace this with parameters.
  std::string urdf_file = "/home/eratner/j2s7s300.urdf";
  std::string root_name = "j2s7s300_link_base";
  std::string tip_name  = "j2s7s300_end_effector";

  // Construct the dynamics model from URDF file.
  try
  {
    dynamics_ = new ucb_jaco_control::URDFRobotDynamics<7>(urdf_file, root_name, tip_name);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Failed to construct dynamics: " << e.what());
    return false;
  }

  // TODO: Allow client to specify desired trajectory somehow.
  Eigen::Matrix<double, 7, 1> setpoint;
  // setpoint << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2, 0.0;
  setpoint << 0, 0, 0, 0, 0, 0, 0;
  desired_trajectory_ = new ConstantTrajectory<7>(setpoint);

  // Construct the controller using this dynamics model.
  std::array<double, 7> default_proportional_gain =
    {P_GAIN, P_GAIN, P_GAIN, P_GAIN, P_GAIN, P_GAIN, P_GAIN};
  std::array<double, 7> default_integral_gain =
    {I_GAIN, I_GAIN, I_GAIN, I_GAIN, I_GAIN, I_GAIN, I_GAIN};
  std::array<double, 7> default_derivative_gain =
    {D_GAIN, D_GAIN, D_GAIN, D_GAIN, D_GAIN, D_GAIN, D_GAIN};

  controller_ = new PIDComputedTorqueController<7>(default_proportional_gain,
                                                   default_integral_gain,
                                                   default_derivative_gain,
                                                   dynamics_,
                                                   desired_trajectory_);

  // TODO: Implement a topic/service for the commanded setpoint.
  // PIDRegulationController<7>::StateVector setpoint;
  // setpoint << 0.0, M_PI_2, 0.0, M_PI_2, 0.0, M_PI_2, 0.0;
  // controller_->setSetpoint(setpoint);

  // Publisher for the errors.
  error_pub_ = nh.advertise<std_msgs::Float64MultiArray>("errors", 1);

  // Dynamic reconfigure for the PID gains.
  server_ = new dynamic_reconfigure::Server<PIDGainsConfig>(nh);
  server_->setCallback(boost::bind(&TestControllerROS::pidGainsCallback, this, _1, _2));

  // ROS_INFO_STREAM("Setpoint is " << controller_.getSetpoint());

  return true;
}

void TestControllerROS::starting(ros::Time& time)
{
  ROS_INFO_STREAM("Starting test controller at time " << time);

  controller_->reset();
}

void TestControllerROS::update(const ros::Time& time, const ros::Duration& period)
{
  // PIDRegulationController<7>::AugmentedStateVector state;
  PIDComputedTorqueController<7>::StateVector state;
  for (int i = 0; i < 7; ++i)
    state(i) = joint_handle_[i].getPosition();
  for (int i = 0; i < 7; ++i)
    state(i + 7) = joint_handle_[i].getVelocity();

  const double t = time.toSec();
  // PIDRegulationController<7>::ControlVector control = controller_->getControl(state, dt);
  PIDComputedTorqueController<7>::ControlVector control = controller_->getControl(state, t);

  const Eigen::Matrix<double, 7, 1>& error = controller_->getError();

  ROS_INFO("%%%%%%%%%%");
  ROS_INFO("\tError \tControl");
  for (int i = 0; i < 7; ++i)
  {
    ROS_INFO_STREAM("\t" << error(i) << " \t" << control(i));
  }

  for (int i = 0; i < 7; ++i)
    joint_handle_[i].setCommand(control(i));

  // Publish the errors.
  std_msgs::Float64MultiArray error_msg;

  for (int i = 0; i < 7; ++i)
    error_msg.data.push_back(error(i));

  error_pub_.publish(error_msg);
}

void TestControllerROS::stopping(const ros::Time& time)
{
  ROS_INFO_STREAM("Stopping test controller at time " << time);
}

void TestControllerROS::pidGainsCallback(PIDGainsConfig& config, uint32_t level)
{
  ROS_INFO_STREAM("Reconfiguring with proportional gain: " << config.p_gain <<
                  ", integral gain: " << config.i_gain << ", derivative gain: " <<
                  config.d_gain);

  std::array<double, 7> p_gain;
  p_gain.fill(config.p_gain);
  controller_->setProportionalGain(p_gain);

  std::array<double, 7> i_gain;
  i_gain.fill(config.i_gain);
  controller_->setIntegralGain(i_gain);

  std::array<double, 7> d_gain;
  d_gain.fill(config.d_gain);
  controller_->setDerivativeGain(d_gain);
}

} // namespace ucb_jaco_control

PLUGINLIB_EXPORT_CLASS(ucb_jaco_control::TestControllerROS,
                       controller_interface::ControllerBase)
