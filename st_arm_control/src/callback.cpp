#include "callback.h"

extern rmd_motor _BASE_MC[4];
extern Dynamics::JMDynamics jm_dynamics;
extern Mobile_Base base_ctrl;


Callback::Callback() {}


void Callback::SwitchMode(const std_msgs::Int32ConstPtr &msg)
{
  jm_dynamics.SwitchMode(msg);
}

void Callback::HMDTFCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  jm_dynamics.hmd_position.x() = msg->pose.position.x;
  jm_dynamics.hmd_position.y() = msg->pose.position.y;
  jm_dynamics.hmd_position.z() = msg->pose.position.z;

  jm_dynamics.hmd_quaternion.x() = msg->pose.orientation.x;
  jm_dynamics.hmd_quaternion.y() = msg->pose.orientation.y;
  jm_dynamics.hmd_quaternion.z() = msg->pose.orientation.z;
  jm_dynamics.hmd_quaternion.w() = msg->pose.orientation.w;
}

void Callback::SwitchGainTaskSpaceP(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  jm_dynamics.gain_p_task_space[0] = msg->data.at(0);
  jm_dynamics.gain_p_task_space[1] = msg->data.at(1);
  jm_dynamics.gain_p_task_space[2] = msg->data.at(2);
}

void Callback::SwitchGainTaskSpaceW(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  jm_dynamics.gain_w_task_space[0] = msg->data.at(0);
  jm_dynamics.gain_w_task_space[1] = msg->data.at(1);
  jm_dynamics.gain_w_task_space[2] = msg->data.at(2);
}

void Callback::SwitchGainP(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  for(uint8_t i = 0; i < 7; i++) jm_dynamics.gain_p_joint_space[i] = msg -> data.at(i);
}

void Callback::SwitchGainD(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  for(uint8_t i = 0; i < 7; i++) jm_dynamics.gain_d_joint_space[i] = msg -> data.at(i);
}

void Callback::SwitchGainR(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  for(uint8_t i = 0; i < 6; i++) jm_dynamics.gain_r[i] = msg -> data.at(i);
}

void Callback::InitializePose(const std_msgs::BoolConstPtr &msg)
{
  if(msg->data) for(uint8_t i=0; i<6; i++) _BASE_MC[i].initialize_position = true;
  std::cout << "Initialized Pose" << std::endl;
}

void Callback::GripperCallback(const std_msgs::Float32ConstPtr &msg)
{
  jm_dynamics.SetGripperValue(msg->data);
}

void Callback::WheelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  Vector3d temp_base_vel;
  temp_base_vel << msg->linear.x, msg->linear.y, msg->angular.z;

  base_ctrl.SetBaseVelocity(temp_base_vel);

  std::cout << "SetBaseVelocity!!" <<  std::endl;

  std::cout << "    FL speed: " <<  base_ctrl.ref_wheel_speed[0];
  std::cout << "    BL speed: " <<  base_ctrl.ref_wheel_speed[1];
  std::cout << "    FR speed: " <<  base_ctrl.ref_wheel_speed[2];
  std::cout << "    BR speed: " <<  base_ctrl.ref_wheel_speed[3] << std::endl;

  std::cout << "    x(forward)" << temp_base_vel[0] 
            << "    y(right):"  << temp_base_vel[1]
            << "    z(yaw):"    << temp_base_vel[2] << std::endl;
}

void Callback::JoystickCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  Vector3d joystick_value;
  joystick_value << msg -> data.at(0), msg -> data.at(1), msg -> data.at(2);

  base_ctrl.SetJoystickValue(joystick_value);

  std::cout << "SetJoystickValue!!" <<  std::endl;

  std::cout << "    x(forward)" <<  base_ctrl.maped_velocity[0];
  std::cout << "    y(right):"  <<  base_ctrl.maped_velocity[1];
  std::cout << "    z(yaw):"    <<  base_ctrl.maped_velocity[2] << std::endl;

  std::cout << "    FL speed: " <<  base_ctrl.ref_wheel_speed[0];
  std::cout << "    BL speed: " <<  base_ctrl.ref_wheel_speed[1];
  std::cout << "    FR speed: " <<  base_ctrl.ref_wheel_speed[2];
  std::cout << "    BR speed: " <<  base_ctrl.ref_wheel_speed[3] << std::endl;
}


void Callback::JoystickCallbackPS5(const sensor_msgs::JoyConstPtr &msg)
{
  Vector3d joystick_value;
  joystick_value << msg->axes.at(1), msg->axes.at(0), msg->axes.at(3);
  base_ctrl.SetJoystickValue(joystick_value);
}