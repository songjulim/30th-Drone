#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "main.h"

extern volatile float sensor_gyro_x_dps;
extern volatile float sensor_gyro_y_dps;
extern volatile float sensor_gyro_z_dps;
extern volatile float sensor_accel_x_g;
extern volatile float sensor_accel_y_g;
extern volatile float sensor_accel_z_g;
extern volatile float sensor_roll_deg;
extern volatile float sensor_pitch_deg;
extern volatile float sensor_yaw_deg;
extern volatile int imuimu;

HAL_StatusTypeDef sensor_init(void);
void sensor_process(void);
void sensor_get_attitude(float *roll_deg, float *pitch_deg, float *yaw_deg);

#endif
