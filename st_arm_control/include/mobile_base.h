#ifndef MOBILE_BASE_H
#define MOBILE_BASE_H

#include <eigen3/Eigen/Dense>
#define   dist_A     25
#define   dist_B     20
#define   wheel_diameter    15            //cm 

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

    void SetActualWheelSpeed(VectorXd a_wheel_speed);
    VectorXd GetTargetWheelSpeed();
    void SetBaseVelocity(Vector3d);
    void BaseMovingGeneration();
    

private:

};
#endif  // MOBILE_BASE_H