#ifndef RMD_MOTOR_H
#define RMD_MOTOR_H

#include <linux/types.h>
#include <math.h>
#include "rmd_can.h"
#include "rt_utils.h"
#include "dynamics.h"

class rmd_motor
{
public:
    rmd_motor();

    unsigned char ref_data[8];
    // unsigned char ref_data2[8];
    unsigned char rx_data_a1[8];
    unsigned char rx_data_a2[8];
    unsigned char enc_data_v3[8];

    // float temp_theta = temp_encoder;
    int     count;
    // int     count_92;
    // int     count_A1;
    // float temp_theta = temp_encoder;
    int     count_A2;
    int     unknown_value;

    int     actuator_gear_ratio;
    int     actuator_direction;
    float   actuator_torque_limit;
    float   actuator_speed_limit;
    float   joint_initial_position;
    bool    initialize_position = true;
    float   torque_to_data;
    float   speed_to_data;
    float   data_to_radian;

    void    UpdateRxDataA1(void);
    void    UpdateRxDataA2(void);
    // void    SetTorqueData(void);
    float   GetTheta();
    float   GetThetaV3();
    float   GetThetaDot();
    float   GetTorque();
    float   GetSpeed();

private:
    float   joint_velocity;
    float   joint_theta;
    float   joint_torque;
    float   joint_temperature;

    float   motor_theta_last;

    float   joint_theta_92;
    float   joint_theta_offset_92;
};

#endif // RMD_MOTOR_H
