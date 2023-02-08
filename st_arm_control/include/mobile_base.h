#ifndef MOBILE_BASE_H
#define MOBILE_BASE_H


#include "motor_controller.h"
#include <eigen3/Eigen/Dense>
#define   dist_A     0.25                  //m
#define   dist_B     0.2                   //m
#define   wheel_diameter    0.15           //m 

using Eigen::Quaterniond;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

class Mobile_Base
{
public:
    Mobile_Base();
    ~Mobile_Base();

    Quaterniond joystick_quaternion;
    VectorXd joystick_position = VectorXd::Zero(3);
    VectorXd wheel_speed = VectorXd::Zero(4);         //wheel 4
    VectorXd ref_wheel_speed = VectorXd::Zero(4);     //wheel 4
    MatrixXd Jacobian_Wheel = MatrixXd::Zero(4,3);
    Vector3d ref_base_pos_dot = VectorXd::Zero(3);
    Vector3d joystick_value = VectorXd::Zero(3);  //unmaped joystick_value
    Vector3d maped_velocity = VectorXd::Zero(3);    //maped joystick_value

    void SetActualWheelSpeed(VectorXd a_wheel_speed);
    VectorXd GetTargetWheelSpeed();
    void SetBaseVelocity(Vector3d);
    void BaseMovingGeneration();
    void SetJoystickValue(Vector3d a_joystick_value);
    Vector3d GetJoystickValue();
    Vector3d MapJoy2Vel(float joy_max, float joy_min, float vel_max, float vel_min, Vector3d a_joystick_value);
    void Loop();

    

private:

};
#endif  // MOBILE_BASE_H