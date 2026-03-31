#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

extern volatile float motor_target_roll_rate_dps;
extern volatile float motor_target_pitch_rate_dps;
extern volatile float motor_target_yaw_rate_dps;

void motor_init(TIM_HandleTypeDef *htim);
void motor_set_channels(uint32_t channel_1_compare,
                        uint32_t channel_2_compare,
                        uint32_t channel_3_compare,
                        uint32_t channel_4_compare);
void motor_set_all(uint32_t compare);
void motor_set_throttle(uint32_t compare);
void motor_set_rate_targets(float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps);
void motor_set_rate_pid_gains(float roll_kp,
                              float roll_kd,
                              float pitch_kp,
                              float pitch_kd,
                              float yaw_kp,
                              float yaw_kd);
void motor_reset_rate_pid(void);
void motor_rate_pid_update(void);
void motor_stop(void);

#endif
