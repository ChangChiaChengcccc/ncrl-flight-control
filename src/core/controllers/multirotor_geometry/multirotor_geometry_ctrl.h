#ifndef __MULTIROTOR_GEOMETRY_CTRL_H__
#define __MULTIROTOR_GEOMETRY_CTRL_H__

#include "imu.h"
#include "ahrs.h"
#include "debug_link.h"
#include "sbus_radio.h"

extern float ukf_pos_enu[3]; 
extern float ukf_vel_enu[3];
extern float ukf_acc_enu[3];
extern float ukf_W[3];
extern float ukf_f1_cmd;
extern float ukf_f2_cmd;
extern float ukf_f3_cmd;
extern float ukf_f4_cmd;
extern float ukf_RotMat_array[9];

void geometry_ctrl_init(void);
void multirotor_geometry_control(radio_t *rc, float *desired_heading);

void send_geometry_moment_ctrl_debug(debug_msg_t *payload);
void send_geometry_tracking_ctrl_debug(debug_msg_t *payload);
void send_uav_dynamics_debug(debug_msg_t *payload);

#endif
