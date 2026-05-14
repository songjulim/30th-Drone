#include "motor.h"
#include "switch.h"
#include "sensor.h"
#include "oled.h"
#include <stddef.h>

static TIM_HandleTypeDef *motor_tim = NULL;

#define MOTOR_OUTPUT_MIN_COMPARE     1000U
#define MOTOR_OUTPUT_MAX_COMPARE     2000U
#define MOTOR_RATE_PID_DT_SECONDS    (1.0f / 2000.0f)
#define MOTOR_RATE_PID_OUTPUT_LIMIT  400.0f

volatile float motor_target_roll_rate_dps = 0.0f;
volatile float motor_target_pitch_rate_dps = 0.0f;
volatile float motor_target_yaw_rate_dps = 0.0f;
volatile float motor_target_roll_angle_deg = 0.0f;
volatile float motor_target_pitch_angle_deg = 0.0f;

typedef struct
{
  float kp;
  float kd;
} motor_angle_pd_axis_t;

typedef struct
{
  float kp;
  float kd;
  float previous_measurement_dps;
  uint8_t initialized;
} motor_rate_pid_axis_t;

static motor_rate_pid_axis_t roll_rate_pid =
{
  .kp = 0.00f,      //3.00f
  .kd = 0.00f,     //0.008f
  .previous_measurement_dps = 0.0f,
  .initialized = 0U
};

static motor_rate_pid_axis_t pitch_rate_pid =
{
  .kp = 0.00f,      //4.00f
  .kd = 0.00f,     //0.005f
  .previous_measurement_dps = 0.0f,
  .initialized = 0U
};

static motor_rate_pid_axis_t yaw_rate_pid =
{
  .kp = 0.00f,
  .kd = 0.00f,
  .previous_measurement_dps = 0.0f,
  .initialized = 0U
};

static motor_angle_pd_axis_t roll_angle_pd =
{
  .kp = 0.00f,
  .kd = 0.00f
};

static motor_angle_pd_axis_t pitch_angle_pd =
{
  .kp = 0.00f,
  .kd = 0.00f
};

static uint32_t motor_throttle_compare = MOTOR_OUTPUT_MIN_COMPARE;
static float motor_clamp_float(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static uint32_t motor_clamp_compare(float compare)
{
  if (compare <= (float)MOTOR_OUTPUT_MIN_COMPARE)
  {
    return MOTOR_OUTPUT_MIN_COMPARE;
  }

  if (compare >= (float)MOTOR_OUTPUT_MAX_COMPARE)
  {
    return MOTOR_OUTPUT_MAX_COMPARE;
  }

  return (uint32_t)compare;
}

static float motor_rate_pid_step(motor_rate_pid_axis_t *pid,
                                 float target_rate_dps,
                                 float measured_rate_dps)
{
  float error_dps;
  float derivative_dps_per_second;
  float output;

  if (pid == NULL)
  {
    return 0.0f;
  }

  error_dps = target_rate_dps - measured_rate_dps;

  if (pid->initialized == 0U)
  {
    pid->previous_measurement_dps = measured_rate_dps;
    pid->initialized = 1U;
  }

  derivative_dps_per_second =
      (measured_rate_dps - pid->previous_measurement_dps) / MOTOR_RATE_PID_DT_SECONDS;
  pid->previous_measurement_dps = measured_rate_dps;

  output = (pid->kp * error_dps) -
           (pid->kd * derivative_dps_per_second);

  return motor_clamp_float(output,
                           -MOTOR_RATE_PID_OUTPUT_LIMIT,
                            MOTOR_RATE_PID_OUTPUT_LIMIT);
}

static float motor_angle_pd_step(const motor_angle_pd_axis_t *pd,
                                 float target_angle_deg,
                                 float measured_angle_deg,
                                 float measured_rate_dps)
{
  float error_deg;

  if (pd == NULL)
  {
    return 0.0f;
  }

  error_deg = target_angle_deg - measured_angle_deg;

  return (pd->kp * error_deg) -
         (pd->kd * measured_rate_dps);
}

static void motor_set_compare(uint32_t channel, uint32_t compare)
{
  if (motor_tim == NULL)
  {
    return;
  }

  __HAL_TIM_SET_COMPARE(motor_tim, channel, compare);
}

void motor_init(TIM_HandleTypeDef *htim)
{
  if (htim == NULL)
  {
    return;
  }

  motor_tim = htim;
  motor_reset_rate_pid();

  (void)HAL_TIM_PWM_Start(motor_tim, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(motor_tim, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Start(motor_tim, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Start(motor_tim, TIM_CHANNEL_4);


  OLED_Clear();
  OLED_Printf(2, 0, "MOTOR Setting");
  OLED_Printf(4, 0, "Press UP");
  OLED_Update();

  while(1)
  {
    switch_update();
    if(sw_u_flag==1)
    {
      OLED_Clear();
      OLED_Printf(4, 0, "Setting...");
      OLED_Update();
      motor_set_all(2000U);
      HAL_Delay(3000);
      motor_set_all(1000U);
      HAL_Delay(3500);
      OLED_Clear();
      OLED_Printf(4, 0, "Finished!");
      OLED_Update();
      HAL_Delay(1500);
      break;
    }
  }
  motor_stop();
}

void motor_set_all(uint32_t compare)
{
  motor_set_channels(compare, compare, compare, compare);
}

void motor_set_throttle(uint32_t compare)
{
  motor_throttle_compare = motor_clamp_compare((float)compare);
}

void motor_set_channels(uint32_t channel_1_compare,
                        uint32_t channel_2_compare,
                        uint32_t channel_3_compare,
                        uint32_t channel_4_compare)
{
  motor_set_compare(TIM_CHANNEL_1, channel_1_compare);
  motor_set_compare(TIM_CHANNEL_2, channel_2_compare);
  motor_set_compare(TIM_CHANNEL_3, channel_3_compare);
  motor_set_compare(TIM_CHANNEL_4, channel_4_compare);
}

void motor_set_rate_targets(float roll_rate_dps, float pitch_rate_dps, float yaw_rate_dps)
{
  motor_target_roll_rate_dps = roll_rate_dps;
  motor_target_pitch_rate_dps = pitch_rate_dps;
  motor_target_yaw_rate_dps = yaw_rate_dps;
}

void motor_set_angle_targets(float roll_deg, float pitch_deg)
{
  motor_target_roll_angle_deg = roll_deg;
  motor_target_pitch_angle_deg = pitch_deg;
}

void motor_set_angle_pd_gains(float roll_kp,
                              float roll_kd,
                              float pitch_kp,
                              float pitch_kd)
{
  roll_angle_pd.kp = roll_kp;
  roll_angle_pd.kd = roll_kd;
  pitch_angle_pd.kp = pitch_kp;
  pitch_angle_pd.kd = pitch_kd;
}

void motor_set_rate_pid_gains(float roll_kp,
                              float roll_kd,
                              float pitch_kp,
                              float pitch_kd,
                              float yaw_kp,
                              float yaw_kd)
{
  roll_rate_pid.kp = roll_kp;
  roll_rate_pid.kd = roll_kd;
  pitch_rate_pid.kp = pitch_kp;
  pitch_rate_pid.kd = pitch_kd;
  yaw_rate_pid.kp = yaw_kp;
  yaw_rate_pid.kd = yaw_kd;
}

void motor_reset_rate_pid(void)
{
  roll_rate_pid.previous_measurement_dps = 0.0f;
  roll_rate_pid.initialized = 0U;

  pitch_rate_pid.previous_measurement_dps = 0.0f;
  pitch_rate_pid.initialized = 0U;

  yaw_rate_pid.previous_measurement_dps = 0.0f;
  yaw_rate_pid.initialized = 0U;

  motor_target_roll_rate_dps = 0.0f;
  motor_target_pitch_rate_dps = 0.0f;
  motor_target_yaw_rate_dps = 0.0f;
  motor_target_roll_angle_deg = 0.0f;
  motor_target_pitch_angle_deg = 0.0f;
}

void motor_rate_pid_update(void)
{
  float roll_output;
  float pitch_output;
  float yaw_output;
  float effective_roll_rate_dps;
  float effective_pitch_rate_dps;
  float base_compare;

  base_compare = (float)motor_throttle_compare;

  effective_roll_rate_dps = motor_target_roll_rate_dps +
                            motor_angle_pd_step(&roll_angle_pd,
                                                motor_target_roll_angle_deg,
                                                sensor_roll_deg,
                                                sensor_gyro_x_dps);
  effective_pitch_rate_dps = motor_target_pitch_rate_dps +
                             motor_angle_pd_step(&pitch_angle_pd,
                                                 motor_target_pitch_angle_deg,
                                                 sensor_pitch_deg,
                                                 sensor_gyro_y_dps);

  roll_output = motor_rate_pid_step(&roll_rate_pid,
                                    effective_roll_rate_dps,
                                    sensor_gyro_x_dps);
  pitch_output = motor_rate_pid_step(&pitch_rate_pid,
                                     effective_pitch_rate_dps,
                                     sensor_gyro_y_dps);
  yaw_output = motor_rate_pid_step(&yaw_rate_pid,
                                   motor_target_yaw_rate_dps,
                                   sensor_gyro_z_dps);

  motor_set_channels(motor_clamp_compare(base_compare + pitch_output - roll_output + yaw_output),
                     motor_clamp_compare(base_compare + pitch_output + roll_output - yaw_output),
                     motor_clamp_compare(base_compare - pitch_output + roll_output + yaw_output),
                     motor_clamp_compare(base_compare - pitch_output - roll_output - yaw_output));
}

void motor_stop(void)
{
  motor_set_throttle(MOTOR_OUTPUT_MIN_COMPARE);
  motor_reset_rate_pid();
  motor_set_all(MOTOR_OUTPUT_MIN_COMPARE);
}
