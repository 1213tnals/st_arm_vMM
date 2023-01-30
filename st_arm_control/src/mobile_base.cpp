#include "mobile_base.h"


Mobile_Base::Mobile_Base() {}
Mobile_Base::~Mobile_Base() {}


void Mobile_Base::SetActualWheelSpeed(VectorXd a_wheel_speed)
{
    for(uint8_t i=0; i<4; i++) wheel_speed[i] = a_wheel_speed[i];
}

VectorXd Mobile_Base::GetTargetWheelSpeed()
{
    return ref_wheel_speed;
}

void Mobile_Base::SetBaseVelocity(Vector3d a_ref_base_velocity)
{
    for(uint8_t i=0; i<3; i++) ref_base_pos_dot[i] = a_ref_base_velocity[i];
}

void Mobile_Base::BaseMovingGeneration()
{
    // ref_base_pos_dot << joystick_position.y(), -joystick_position.x(), 0;

    Jacobian_Wheel << 1, -1, -(dist_A + dist_B),
                      1,  1,  (dist_A + dist_B),
                      1,  1, -(dist_A + dist_B),
                      1, -1,  (dist_A + dist_B);

    ref_wheel_speed = 2 / wheel_diameter * Jacobian_Wheel * ref_base_pos_dot;
}