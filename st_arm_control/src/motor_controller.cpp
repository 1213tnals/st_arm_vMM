#include "motor_controller.h"
#define   wheel_diameter    15            //cm
// #define   rad2deg           57.2957802

extern pRBCORE_SHM sharedData;
extern rmd_motor _BASE_MC[num_of_rmdX];
extern Dynamixel _WRIST_MC;

Motor_Controller::Motor_Controller()
{
  // _BASE_MC[0].actuator_direction = -1;    _BASE_MC[0].actuator_gear_ratio = 8;  _BASE_MC[0].joint_initial_position = 0.0;       
  // _BASE_MC[0].torque_to_data = 140;    _BASE_MC[0].actuator_torque_limit = 4.5*2;      _BASE_MC[0].data_to_radian = tic2radL; // 10430.2197
  // _BASE_MC[0].actuator_direction = -1;    _BASE_MC[0].actuator_gear_ratio = 6;  _BASE_MC[0].joint_initial_position = 0.0;       
  // _BASE_MC[0].torque_to_data = 225;    _BASE_MC[0].actuator_torque_limit = 4.5*2;      _BASE_MC[0].data_to_radian = tic2radL; // 10430.2197
  _BASE_MC[0].actuator_direction = 1;   _BASE_MC[0].actuator_gear_ratio = 6;  _BASE_MC[0].joint_initial_position = 0.0;
  _BASE_MC[0].torque_to_data = 225;     _BASE_MC[0].actuator_torque_limit = 4.5*2;      _BASE_MC[0].data_to_radian = tic2radX;   //53.42
  _BASE_MC[0].speed_to_data = rad2deg/wheel_diameter*100;      _BASE_MC[0].actuator_speed_limit = 30;
  
  _BASE_MC[1].actuator_direction = 1;   _BASE_MC[1].actuator_gear_ratio = 6;  _BASE_MC[1].joint_initial_position = 0.0;
  _BASE_MC[1].torque_to_data = 225;    _BASE_MC[1].actuator_torque_limit = 4.5*2;      _BASE_MC[1].data_to_radian = tic2radX;
  _BASE_MC[1].speed_to_data = rad2deg/wheel_diameter*100;      _BASE_MC[1].actuator_speed_limit = 30;
  
  _BASE_MC[2].actuator_direction = -1;  _BASE_MC[2].actuator_gear_ratio = 6;  _BASE_MC[2].joint_initial_position = 0.0;
  _BASE_MC[2].torque_to_data = 225;    _BASE_MC[2].actuator_torque_limit = 4.5*2;      _BASE_MC[2].data_to_radian = tic2radX;
  _BASE_MC[2].speed_to_data = rad2deg/wheel_diameter*100;      _BASE_MC[0].actuator_speed_limit = 30;
 
  _BASE_MC[3].actuator_direction = -1;  _BASE_MC[3].actuator_gear_ratio = 6;  _BASE_MC[3].joint_initial_position = 0.0;
  _BASE_MC[3].torque_to_data = 225;    _BASE_MC[3].actuator_torque_limit = 4.5*2;      _BASE_MC[3].data_to_radian = tic2radX;
  _BASE_MC[3].speed_to_data = rad2deg/wheel_diameter*100;      _BASE_MC[3].actuator_speed_limit = 30;
}

void Motor_Controller::EnableMotor(){
  sharedData->rmd_motor_run_flag[0] = false;   // 0xA2
  sharedData->rmd_motor_run_flag[1] = false;
  sharedData->rmd_motor_run_flag[2] = false;
  sharedData->rmd_motor_run_flag[3] = false;
  sharedData->rmd_motor_run_flag[4] = false;
  sharedData->rmd_motor_run_flag[5] = false;
  sharedData->rmd_motor_run_flag[6] = true;    // FL
  sharedData->rmd_motor_run_flag[7] = true;    // BL
  sharedData->rmd_motor_run_flag[8] = false;
  sharedData->rmd_motor_run_flag[9] = true;    // FR
  sharedData->rmd_motor_run_flag[10] = true;   // BR
  sharedData->rmd_motor_run_flag[11] = false;
  
  for(uint8_t i=0; i<num_of_rmdX; i++) {
    _BASE_MC[i].initialize_position = true;
    _BASE_MC[i].ref_data[0] = 0x88 & 0xFF;
    _BASE_MC[i].ref_data[1] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[2] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[3] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[4] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[5] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[6] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[7] = 0x00 & 0xFF;
  }
}


VectorXd Motor_Controller::GetJointTheta(){
  // for(uint8_t i=0; i<3; i++) th_joint[i] = _BASE_MC[i].GetTheta();
  VectorXd th_(num_of_dynamixels);
  th_ = _WRIST_MC.GetThetaAct();
  for(uint8_t i=0;i<num_of_dynamixels; i++) th_joint[i] = th_[i];

  return th_joint;
}


VectorXd Motor_Controller::GetThetaDot(){
  // for(uint8_t i=0; i<3; i++) th_dot[i] = _BASE_MC[i].GetThetaDot();
  VectorXd th_dot_(num_of_dynamixels);
  th_dot_ = _WRIST_MC.GetThetaDot();
  for(uint8_t i=0; i<num_of_dynamixels; i++) th_dot[i] = th_dot_[i];
  // th_dot[0] = th_dot[0] / 6;
  return th_dot;
}


// VectorXd Motor_Controller::GetThetaDotEst(){
//   VectorXd th_dot_est_(num_of_dynamixels); th_dot_est_ = _WRIST_MC.GetThetaDotEstimated();
//   return th_dot_est_;
// }

// Simple Moving Average filtered Joint Velocity
VectorXd Motor_Controller::GetThetaDotSMAF(){
  // for(uint8_t i=0; i<3; i++) th_dot[i] = _BASE_MC[i].GetThetaDot();    // Get from shoulder motors RMD-X6
  // VectorXd th_dot_(4); th_dot_ = _WRIST_MC.GetThetaDot(); for(uint8_t i=0; i<4; i++) th_dot[i+3] = th_dot_[i];   // Get from wrist motors: Dynamixel from motor
  VectorXd a_th_dot(num_of_dynamixels); a_th_dot = _WRIST_MC.GetThetaDotEstimated(); for(uint8_t i=0; i<num_of_dynamixels; i++) th_dot[i] = a_th_dot[i];   // Get from wrist motors: Dynamixel from estimated
  // th_dot[0] = th_dot[0] / 6; // Because V2 motor driver

  sma << sma.block<window_size-1, num_of_dynamixels>(1, 0), th_dot[0], th_dot[1], th_dot[2], th_dot[3], th_dot[4], th_dot[5], th_dot[6];
  th_dot_sma_filtered = sma.colwise().mean();

  return th_dot_sma_filtered;
}


VectorXd Motor_Controller::GetTorque(){
  VectorXd tau(num_of_dynamixels); 
  // for(uint8_t i=0;i<3;i++)
  // {
  //   tau[i] = _BASE_MC[i].GetTorque();
  // }
  return tau;
}

VectorXd Motor_Controller::GetWheelSpeed(){
  VectorXd wheel_speed(num_of_rmdX); 
  for(uint8_t i=0;i<num_of_rmdX;i++)
  {
    wheel_speed[i] = _BASE_MC[i].GetSpeed();
  }
  return wheel_speed;
}


void Motor_Controller::SetTorque(VectorXd tau){
  // for(uint8_t i=0; i<3; i++) {
  //   if (true) {
  //     if (tau[i] > _BASE_MC[i].actuator_torque_limit) tau[i] = _BASE_MC[i].actuator_torque_limit;
  //     if (tau[i] < -1 * _BASE_MC[i].actuator_torque_limit) tau[i] = -1 * _BASE_MC[i].actuator_torque_limit;
  //   }
  //   long param = _BASE_MC[i].actuator_direction * _BASE_MC[i].torque_to_data * tau[i];
  //   _BASE_MC[i].ref_data[0] = 0xa1 & 0xFF;
  //   _BASE_MC[i].ref_data[1] = 0x00 & 0xFF;
  //   _BASE_MC[i].ref_data[2] = 0x00 & 0xFF;
  //   _BASE_MC[i].ref_data[3] = 0x00 & 0xFF;
  //   _BASE_MC[i].ref_data[4] = (param     ) & 0xFF;
  //   _BASE_MC[i].ref_data[5] = (param >> 8) & 0xFF;
  //   _BASE_MC[i].ref_data[6] = 0x00 & 0xFF;
  //   _BASE_MC[i].ref_data[7] = 0x00 & 0xFF;
  // }

  for(uint8_t i=0; i<num_of_dynamixels; i++) torque_wrist[i] = tau[i];
  _WRIST_MC.SetTorqueRef(torque_wrist);
}

void Motor_Controller::SetWheelSpeed(VectorXd ref_wheel_speed){
  for(uint8_t i=0; i<num_of_rmdX; i++) {
    if (true) {
      if (ref_wheel_speed[i] > _BASE_MC[i].actuator_speed_limit) ref_wheel_speed[i] = _BASE_MC[i].actuator_speed_limit;
      if (ref_wheel_speed[i] < -1 * _BASE_MC[i].actuator_speed_limit) ref_wheel_speed[i] = -1 * _BASE_MC[i].actuator_speed_limit;
    }
    uint32_t param = _BASE_MC[i].actuator_direction * _BASE_MC[i].speed_to_data * ref_wheel_speed[i];
    _BASE_MC[i].ref_data[0] = 0xa2 & 0xFF;
    _BASE_MC[i].ref_data[1] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[2] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[3] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[4] = (param      ) & 0xFF;
    _BASE_MC[i].ref_data[5] = (param >> 8 ) & 0xFF;
    _BASE_MC[i].ref_data[6] = (param >> 16) & 0xFF;
    _BASE_MC[i].ref_data[7] = (param >> 24) & 0xFF;
  }
}

// void Motor_Controller::ReadTheta(){
//   for(uint8_t i=0; i<3; i++) {
//     _BASE_MC[i].ref_data2[0] = 0x92 & 0xFF;
//     _BASE_MC[i].ref_data2[1] = 0x00 & 0xFF;
//     _BASE_MC[i].ref_data2[2] = 0x00 & 0xFF;
//     _BASE_MC[i].ref_data2[3] = 0x00 & 0xFF;
//     _BASE_MC[i].ref_data2[4] = 0x00 & 0xFF;
//     _BASE_MC[i].ref_data2[5] = 0x00 & 0xFF;
//     _BASE_MC[i].ref_data2[6] = 0x00 & 0xFF;
//     _BASE_MC[i].ref_data2[7] = 0x00 & 0xFF;
//   }
// }

// 0x141 0x20 0x02 0x00 0x00 0x01 0x00 0x00 0x00
void Motor_Controller::EnableFilter(){
  for(uint8_t i=0; i<num_of_rmdX; i++) {
    _BASE_MC[i].ref_data[0] = 0x20 & 0xFF;
    _BASE_MC[i].ref_data[1] = 0x02 & 0xFF;
    _BASE_MC[i].ref_data[2] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[3] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[4] = 0x01 & 0xFF;
    _BASE_MC[i].ref_data[5] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[6] = 0x00 & 0xFF;
    _BASE_MC[i].ref_data[7] = 0x00 & 0xFF;
  }
}