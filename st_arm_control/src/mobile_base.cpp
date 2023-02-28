#include "mobile_base.h"
#include <iostream>

extern Motor_Controller motor_ctrl;

Mobile_Base::Mobile_Base()
{
    Jacobian_Wheel << 1, -1, -(dist_A + dist_B),
                      1,  1, -(dist_A + dist_B),
                      1,  1,  (dist_A + dist_B),
                      1, -1,  (dist_A + dist_B);
}

Mobile_Base::~Mobile_Base() {}


void Mobile_Base::SetActualWheelSpeed(VectorXd a_wheel_speed)
{
    for(uint8_t i=0; i<4; i++) wheel_speed[i] = a_wheel_speed[i];
}

void Mobile_Base::SetBaseVelocity(Vector3d a_ref_base_velocity)
{
    for(uint8_t i=0; i<3; i++) ref_base_pos_dot[i] = a_ref_base_velocity[i];
}

void Mobile_Base::BaseMovingGeneration()
{
    // ref_base_pos_dot << joystick_position.y(), -joystick_position.x(), 0;

    ref_wheel_speed = Jacobian_Wheel * ref_base_pos_dot;

    for(uint8_t i=0; i<4; i++) ref_wheel_speed[i] = ref_wheel_speed[i] * 2 / wheel_diameter ;
}

VectorXd Mobile_Base::GetTargetWheelSpeed()
{
    return ref_wheel_speed;
}

void Mobile_Base::SetJoystickValue(Vector3d a_joystick_value)
{
    for(uint8_t i=0; i<3; i++) joystick_value[i] = a_joystick_value[i];
}

Vector3d Mobile_Base::GetJoystickValue()
{
    return joystick_value;
}

Vector3d Mobile_Base::MapJoy2Vel(float joy_max, float joy_min, float vel_max, float vel_min, Vector3d a_joystick_value)
{
    for(uint8_t i=0; i<3; i++) maped_velocity[i] = (a_joystick_value[i] - joy_min) * (vel_max - vel_min) / (joy_max - joy_min) + vel_min; 
    
    return maped_velocity;
}

void Mobile_Base::GetBaseState()
{

}

void Mobile_Base::SetBaseState()
{
    
}



void Mobile_Base::Loop()
    {
        SetActualWheelSpeed(motor_ctrl.GetWheelSpeed());

        SetBaseVelocity(MapJoy2Vel(1, -1, 0.5, -0.5, GetJoystickValue()));   //x,y,z vel
        BaseMovingGeneration();          //Making Base speed
        motor_ctrl.SetWheelSpeed(GetTargetWheelSpeed());                               //w1,2,3,4 ang_vel
    }

