#include "dynamixel.h"
#define PI                3.141592

Dynamixel::Dynamixel(){
  initActuatorValues();

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  
  if (!portHandler->openPort()) ROS_ERROR("Failed to open the port!");
  if (!portHandler->setBaudRate(BAUDRATE)) ROS_ERROR("Failed to set the baudrate!");
  
  for(uint8_t i=0; i<num_of_dynamixels; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_OperatingMode, 0, &dxl_error);   // current_mode
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("Failed to set torque control mode for Dynamixel ID %d", i);
  }

  // for(uint8_t i=0;i<4;i++) packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_OperatingMode, 3, &dxl_error);   // position_mode

  for(uint8_t i=0; i<num_of_dynamixels; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_TorqueEnable, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i);
  } 

  for(uint8_t i=0; i<num_of_dynamixels; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_LED, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("Failed to enable LED for Dynamixel ID %d", i);
  } 
}


Dynamixel::~Dynamixel(){
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  for(uint8_t i=0; i<num_of_dynamixels; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_TorqueEnable, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("Failed to disable torque for Dynamixel ID %d", i);
  } 

  for(uint8_t i=0; i<num_of_dynamixels; i++)
  {
    packetHandler->write1ByteTxRx(portHandler, dx_id[i], kRegStandard_LED, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) ROS_ERROR("Failed to disable LED for Dynamixel ID %d", i);
  } 
  
  portHandler->closePort();
}


void Dynamixel::Loop(bool RxTh, bool RxThDot, bool TxTorque)
{
  if(RxTh) syncReadTheta();
  if(RxThDot) syncReadThetaDot();
  if(TxTorque) syncWriteTorque();
}


VectorXd Dynamixel::GetThetaAct()
{
  return th_;
}


void Dynamixel::syncReadTheta()
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, kRegStandard_PresentPosition, 4);
  for(uint8_t i=0; i < num_of_dynamixels; i++) groupSyncRead.addParam(dx_id[i]);
  groupSyncRead.fastSyncReadTxRxPacket();
  for(uint8_t i=0; i < num_of_dynamixels; i++) position[i] = groupSyncRead.getData(dx_id[i], kRegStandard_PresentPosition, 4);
  groupSyncRead.clearParam();
  for(uint8_t i=0; i < num_of_dynamixels; i++) th_[i] = convertValue2Radian(position[i]) - PI - zero_manual_offset_[i];
}


VectorXd Dynamixel::GetThetaDot()
{
  VectorXd vel_(num_of_dynamixels);
  for(uint8_t i=0; i < num_of_dynamixels; i++) 
  {
    if(velocity[i] > 4294900000) vel_[i] = (velocity[i] - 4294967295) * 0.003816667; //4,294,967,295 = 0xFFFFFFFF   // 1 = 0.229rpm   // 1 = 0.003816667
    else vel_[i] = velocity[i] * 0.003816667;
  }
  return vel_;
}


void Dynamixel::syncReadThetaDot()
{
  dynamixel::GroupSyncRead groupSyncReadThDot(portHandler, packetHandler, kRegStandard_PresentVelocity, 4);
  for (uint8_t i=0; i < num_of_dynamixels; i++) groupSyncReadThDot.addParam(dx_id[i]);
  groupSyncReadThDot.fastSyncReadTxRxPacket();
  for(uint8_t i=0; i < num_of_dynamixels; i++) velocity[i] = groupSyncReadThDot.getData(dx_id[i], kRegStandard_PresentVelocity, 4);
  groupSyncReadThDot.clearParam();
}


void Dynamixel::SetTorqueRef(VectorXd a_torque)
{
  for (uint8_t i=0; i < num_of_dynamixels; i++) ref_torque_[i] = a_torque[i];
  // ref_torque_[3] = 0;
}


void Dynamixel::syncWriteTorque()
{
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, kRegStandard_GoalCurrent, 2);

  uint8_t parameter[num_of_dynamixels] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (uint8_t i=0; i < num_of_dynamixels; i++)
  {
    // ref_torque_value_[i] = ref_torque_[i] * TORQUE_TO_VALUE;
    ref_torque_value_[i] = torqueToValue(ref_torque_[i], i);
    if(ref_torque_value_[i] > 1000) ref_torque_value_[i] = 1000;
    else if(ref_torque_value_[i] < -1000) ref_torque_value_[i] = -1000;
  }
  for (uint8_t i=0; i < num_of_dynamixels; i++){
    getParam(ref_torque_value_[i], parameter);
    groupSyncWrite.addParam(dx_id[i], (uint8_t *)&parameter);
  }
  groupSyncWrite.txPacket();
  groupSyncWrite.clearParam();
  // std::cout << "torque value 1: " << ref_torque_value_[1] << std::endl;
}


// {XM430-W350} => effort to current  / 1.78*i-?    // current to value  / 2.69f
// {XM430-W350} => 500 = 1.345[A] = 2.3941[Nm] => 208.85 = 1[Nm]

// {XM430-W210} => if effort to current = 1.3*i      => 500 = 1.345[A] = 1.7485 Nm => 1[Nm] = 285
// {XM430-W210} => if effort to current = 1.3*i-0.32 => 500 = 1.345[A] = 1.4285 Nm => 1[Nm] = 350
int32_t Dynamixel::torqueToValue(double torque, uint8_t index)
{
  int32_t value_ = int(torque * torque2value[index]);     // XM430-W210
  // int32_t value_ = int(torque * 208.427);  // XM430-W350
  // if(value_ > 1) value_ += 10;       // friction compensation
  // else if(value_ < -1) value_ -= 10;
  return value_;
}


void Dynamixel::SetThetaRef(VectorXd theta)
{
  for (uint8_t i=0; i < num_of_dynamixels; i++) ref_th_[i] = theta[i];
}


void Dynamixel::syncWriteTheta()
{
  dynamixel::GroupSyncWrite gSyncWriteTh(portHandler, packetHandler, kRegStandard_GoalPosition, 4);

  uint8_t parameter[num_of_dynamixels] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  for (uint8_t i=0; i < num_of_dynamixels; i++){
    ref_th_value_ = ref_th_ * RAD_TO_VALUE;
    getParam(ref_th_value_[i], parameter);
    gSyncWriteTh.addParam(dx_id[i], (uint8_t *)&parameter);
  }
  gSyncWriteTh.txPacket();
  gSyncWriteTh.clearParam();
}


void Dynamixel::getParam(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}


float Dynamixel::convertValue2Radian(int32_t value)
{
  float radian = value / RAD_TO_VALUE;
  return radian;
}


VectorXd Dynamixel::GetThetaDotEstimated()
{
  return th_dot_est_;
}


void Dynamixel::CalculateEstimatedThetaDot(int dt_us)
{
  th_dot_est_ = (th_last_ - th_) / (-dt_us * 0.00001);
  th_last_ = th_;
}

void Dynamixel::initActuatorValues()
{
  torque2value[0] = TORQUE_TO_VALUE_XM430_W350;
  torque2value[1] = TORQUE_TO_VALUE_XM540_W270;
  torque2value[2] = TORQUE_TO_VALUE_XM540_W270;
  torque2value[3] = TORQUE_TO_VALUE_XM430_W350;
  torque2value[4] = TORQUE_TO_VALUE_XM430_W350;
  torque2value[5] = TORQUE_TO_VALUE_XM430_W350;
  torque2value[6] = TORQUE_TO_VALUE_XM430_W350;
  torque2value[7] = TORQUE_TO_VALUE_XM540_W270;
  torque2value[8] = TORQUE_TO_VALUE_XM540_W270;

  zero_manual_offset_[0] = 0;
  zero_manual_offset_[1] = 1.3076;
  zero_manual_offset_[2] = -1.3076;
  zero_manual_offset_[3] = 0;
  zero_manual_offset_[4] = 0;
  zero_manual_offset_[5] = 0;
  zero_manual_offset_[6] = 0;
  zero_manual_offset_[7] = -zero_manual_offset_[1];
  zero_manual_offset_[8] = -zero_manual_offset_[2];
}
