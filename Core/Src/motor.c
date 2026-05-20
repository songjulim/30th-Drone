#include "motor.h"
#include "switch.h"
#include "sensor.h"
#include "oled.h"
#include <stddef.h>

static TIM_HandleTypeDef *motor_tim = NULL;

#define MOTOR_OUTPUT_MIN_COMPARE     1000U
#define MOTOR_OUTPUT_MAX_COMPARE     2000U
#define MOTOR_RATE_PID_DT_SECONDS    (1.0f / 2000.0f)
#define MOTOR_RATE_PID_HZ            2000U
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
  float ki;
  float kd;
  float integral_error_dps_seconds;
  float integral_output_limit;
  float previous_measurement_dps;
  uint8_t initialized;
} motor_rate_pid_axis_t;

static motor_rate_pid_axis_t roll_rate_pid =
{
  .kp = 0.62f,
  .ki = 0.006f,
  .kd = 0.000f,
  .integral_error_dps_seconds = 0.0f,
  .integral_output_limit = 15.0f,
  .previous_measurement_dps = 0.0f,
  .initialized = 0U
};

static motor_rate_pid_axis_t pitch_rate_pid =
{
  .kp = 0.90f,      //0.68
  .ki = 0.018f,
  .kd = 0.0022f,
  .integral_error_dps_seconds = 0.0f,
  .integral_output_limit = 15.0f,
  .previous_measurement_dps = 0.0f,
  .initialized = 0U
};

static motor_rate_pid_axis_t yaw_rate_pid =
{
  .kp = 0.5f,     //1.8
  .ki = 0.0f,    //0.02
  .kd = 0.00f,   //0.005
  .integral_error_dps_seconds = 0.0f,
  .integral_output_limit = 20.0f,
  .previous_measurement_dps = 0.0f,
  .initialized = 0U
};

static motor_angle_pd_axis_t roll_angle_pd =
{
  .kp = 3.4f,    
  .kd = 0.01f
};

static motor_angle_pd_axis_t pitch_angle_pd =
{
  .kp = 3.6f,
  .kd = 0.01f
};

static uint32_t motor_throttle_compare = MOTOR_OUTPUT_MIN_COMPARE;
static float motor_throttle_current_compare = (float)MOTOR_OUTPUT_MIN_COMPARE;
static float motor_throttle_start_compare = (float)MOTOR_OUTPUT_MIN_COMPARE;
static float motor_throttle_target_compare = (float)MOTOR_OUTPUT_MIN_COMPARE;
static uint32_t motor_throttle_ramp_step = 0U;
static uint32_t motor_throttle_ramp_total_steps = 0U;
static uint8_t motor_throttle_ramp_active = 0U;
static volatile uint32_t motor_channel_compare[4] =
{
  MOTOR_OUTPUT_MIN_COMPARE,
  MOTOR_OUTPUT_MIN_COMPARE,
  MOTOR_OUTPUT_MIN_COMPARE,
  MOTOR_OUTPUT_MIN_COMPARE
};
static volatile float motor_roll_output = 0.0f;
static volatile float motor_pitch_output = 0.0f;
static volatile float motor_yaw_output = 0.0f;
static volatile float motor_effective_roll_rate_dps = 0.0f;
static volatile float motor_effective_pitch_rate_dps = 0.0f;
static volatile float motor_roll_angle_error_deg = 0.0f;
static volatile float motor_roll_angle_output_dps = 0.0f;
static volatile float motor_roll_rate_error_dps = 0.0f;
static volatile float motor_roll_rate_p_output = 0.0f;
static volatile float motor_roll_rate_i_output = 0.0f;
static volatile float motor_roll_rate_d_output = 0.0f;
static volatile float motor_pitch_angle_error_deg = 0.0f;
static volatile float motor_pitch_angle_output_dps = 0.0f;
static volatile float motor_pitch_rate_error_dps = 0.0f;
static volatile float motor_pitch_rate_p_output = 0.0f;
static volatile float motor_pitch_rate_i_output = 0.0f;
static volatile float motor_pitch_rate_d_output = 0.0f;

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
                                 float measured_rate_dps,
                                 float *error_dps_out,
                                 float *p_output_out,
                                 float *i_output_out,
                                 float *d_output_out)
{
  float error_dps;
  float derivative_dps_per_second;
  float p_output;
  float i_output;
  float d_output;
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

  if (pid->ki > 0.0f)
  {
    float integral_limit = pid->integral_output_limit / pid->ki;

    pid->integral_error_dps_seconds += error_dps * MOTOR_RATE_PID_DT_SECONDS;
    pid->integral_error_dps_seconds = motor_clamp_float(pid->integral_error_dps_seconds,
                                                        -integral_limit,
                                                         integral_limit);
  }

  p_output = pid->kp * error_dps;
  i_output = pid->ki * pid->integral_error_dps_seconds;
  d_output = -(pid->kd * derivative_dps_per_second);

  if (error_dps_out != NULL)
  {
    *error_dps_out = error_dps;
  }

  if (p_output_out != NULL)
  {
    *p_output_out = p_output;
  }

  if (i_output_out != NULL)
  {
    *i_output_out = i_output;
  }

  if (d_output_out != NULL)
  {
    *d_output_out = d_output;
  }

  output = p_output + i_output + d_output;

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
  motor_throttle_current_compare = (float)motor_throttle_compare;
  motor_throttle_start_compare = motor_throttle_current_compare;
  motor_throttle_target_compare = motor_throttle_current_compare;
  motor_throttle_ramp_step = 0U;
  motor_throttle_ramp_total_steps = 0U;
  motor_throttle_ramp_active = 0U;
}

void motor_set_throttle_ramp(uint32_t target_compare, uint32_t duration_ms)
{
  motor_throttle_target_compare = (float)motor_clamp_compare((float)target_compare);

  if (duration_ms == 0U)
  {
    motor_set_throttle((uint32_t)motor_throttle_target_compare);
    return;
  }

  motor_throttle_start_compare = motor_throttle_current_compare;
  motor_throttle_ramp_step = 0U;
  motor_throttle_ramp_total_steps = (duration_ms * MOTOR_RATE_PID_HZ) / 1000U;

  if (motor_throttle_ramp_total_steps == 0U)
  {
    motor_set_throttle((uint32_t)motor_throttle_target_compare);
    return;
  }

  motor_throttle_ramp_active = 1U;
}

void motor_set_channels(uint32_t channel_1_compare,
                        uint32_t channel_2_compare,
                        uint32_t channel_3_compare,
                        uint32_t channel_4_compare)
{
  motor_channel_compare[0] = channel_1_compare;
  motor_channel_compare[1] = channel_2_compare;
  motor_channel_compare[2] = channel_3_compare;
  motor_channel_compare[3] = channel_4_compare;

  motor_set_compare(TIM_CHANNEL_1, channel_1_compare);
  motor_set_compare(TIM_CHANNEL_2, channel_2_compare);
  motor_set_compare(TIM_CHANNEL_3, channel_3_compare);
  motor_set_compare(TIM_CHANNEL_4, channel_4_compare);
}

void motor_get_channels(uint32_t *channel_1_compare,
                        uint32_t *channel_2_compare,
                        uint32_t *channel_3_compare,
                        uint32_t *channel_4_compare)
{
  if (channel_1_compare != NULL)
  {
    *channel_1_compare = motor_channel_compare[0];
  }

  if (channel_2_compare != NULL)
  {
    *channel_2_compare = motor_channel_compare[1];
  }

  if (channel_3_compare != NULL)
  {
    *channel_3_compare = motor_channel_compare[2];
  }

  if (channel_4_compare != NULL)
  {
    *channel_4_compare = motor_channel_compare[3];
  }
}

uint32_t motor_get_throttle(void)
{
  return motor_throttle_compare;
}

void motor_get_control_outputs(float *roll_output,
                               float *pitch_output,
                               float *yaw_output,
                               float *effective_roll_rate_dps,
                               float *effective_pitch_rate_dps)
{
  if (roll_output != NULL)
  {
    *roll_output = motor_roll_output;
  }

  if (pitch_output != NULL)
  {
    *pitch_output = motor_pitch_output;
  }

  if (yaw_output != NULL)
  {
    *yaw_output = motor_yaw_output;
  }

  if (effective_roll_rate_dps != NULL)
  {
    *effective_roll_rate_dps = motor_effective_roll_rate_dps;
  }

  if (effective_pitch_rate_dps != NULL)
  {
    *effective_pitch_rate_dps = motor_effective_pitch_rate_dps;
  }
}

void motor_get_roll_tuning_debug(float *angle_error_deg,
                                 float *angle_output_dps,
                                 float *rate_error_dps,
                                 float *rate_p_output,
                                 float *rate_i_output,
                                 float *rate_d_output)
{
  if (angle_error_deg != NULL)
  {
    *angle_error_deg = motor_roll_angle_error_deg;
  }

  if (angle_output_dps != NULL)
  {
    *angle_output_dps = motor_roll_angle_output_dps;
  }

  if (rate_error_dps != NULL)
  {
    *rate_error_dps = motor_roll_rate_error_dps;
  }

  if (rate_p_output != NULL)
  {
    *rate_p_output = motor_roll_rate_p_output;
  }

  if (rate_i_output != NULL)
  {
    *rate_i_output = motor_roll_rate_i_output;
  }

  if (rate_d_output != NULL)
  {
    *rate_d_output = motor_roll_rate_d_output;
  }
}

void motor_get_pitch_tuning_debug(float *angle_error_deg,
                                  float *angle_output_dps,
                                  float *rate_error_dps,
                                  float *rate_p_output,
                                  float *rate_i_output,
                                  float *rate_d_output)
{
  if (angle_error_deg != NULL)
  {
    *angle_error_deg = motor_pitch_angle_error_deg;
  }

  if (angle_output_dps != NULL)
  {
    *angle_output_dps = motor_pitch_angle_output_dps;
  }

  if (rate_error_dps != NULL)
  {
    *rate_error_dps = motor_pitch_rate_error_dps;
  }

  if (rate_p_output != NULL)
  {
    *rate_p_output = motor_pitch_rate_p_output;
  }

  if (rate_i_output != NULL)
  {
    *rate_i_output = motor_pitch_rate_i_output;
  }

  if (rate_d_output != NULL)
  {
    *rate_d_output = motor_pitch_rate_d_output;
  }
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
                              float roll_ki,
                              float roll_kd,
                              float pitch_kp,
                              float pitch_ki,
                              float pitch_kd,
                              float yaw_kp,
                              float yaw_ki,
                              float yaw_kd)
{
  roll_rate_pid.kp = roll_kp;
  roll_rate_pid.ki = roll_ki;
  roll_rate_pid.kd = roll_kd;
  pitch_rate_pid.kp = pitch_kp;
  pitch_rate_pid.ki = pitch_ki;
  pitch_rate_pid.kd = pitch_kd;
  yaw_rate_pid.kp = yaw_kp;
  yaw_rate_pid.ki = yaw_ki;
  yaw_rate_pid.kd = yaw_kd;
}

void motor_reset_rate_pid(void)
{
  roll_rate_pid.integral_error_dps_seconds = 0.0f;
  roll_rate_pid.previous_measurement_dps = 0.0f;
  roll_rate_pid.initialized = 0U;

  pitch_rate_pid.integral_error_dps_seconds = 0.0f;
  pitch_rate_pid.previous_measurement_dps = 0.0f;
  pitch_rate_pid.initialized = 0U;

  yaw_rate_pid.integral_error_dps_seconds = 0.0f;
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
  float roll_angle_output_dps;
  float roll_rate_error_dps = 0.0f;
  float roll_rate_p_output = 0.0f;
  float roll_rate_i_output = 0.0f;
  float roll_rate_d_output = 0.0f;
  float pitch_angle_output_dps;
  float pitch_rate_error_dps = 0.0f;
  float pitch_rate_p_output = 0.0f;
  float pitch_rate_i_output = 0.0f;
  float pitch_rate_d_output = 0.0f;
  float base_compare;

  if (motor_throttle_ramp_active != 0U)
  {
    motor_throttle_ramp_step++;

    if (motor_throttle_ramp_step >= motor_throttle_ramp_total_steps)
    {
      motor_throttle_current_compare = motor_throttle_target_compare;
      motor_throttle_ramp_active = 0U;
    }
    else
    {
      float ramp_ratio = (float)motor_throttle_ramp_step / (float)motor_throttle_ramp_total_steps;
      motor_throttle_current_compare = motor_throttle_start_compare +
          ((motor_throttle_target_compare - motor_throttle_start_compare) * ramp_ratio);
    }

    motor_throttle_compare = motor_clamp_compare(motor_throttle_current_compare);
  }

  base_compare = (float)motor_throttle_compare;

  roll_angle_output_dps = motor_angle_pd_step(&roll_angle_pd,
                                              motor_target_roll_angle_deg,
                                              sensor_roll_deg,
                                              sensor_gyro_x_dps);
  effective_roll_rate_dps = motor_target_roll_rate_dps + roll_angle_output_dps;
  pitch_angle_output_dps = motor_angle_pd_step(&pitch_angle_pd,
                                               motor_target_pitch_angle_deg,
                                               sensor_pitch_deg,
                                               sensor_gyro_y_dps);
  effective_pitch_rate_dps = motor_target_pitch_rate_dps + pitch_angle_output_dps;

  roll_output = motor_rate_pid_step(&roll_rate_pid,
                                    effective_roll_rate_dps,
                                    sensor_gyro_x_dps,
                                    &roll_rate_error_dps,
                                    &roll_rate_p_output,
                                    &roll_rate_i_output,
                                    &roll_rate_d_output);
  pitch_output = motor_rate_pid_step(&pitch_rate_pid,

                                     effective_pitch_rate_dps,
                                     sensor_gyro_y_dps,
                                     &pitch_rate_error_dps,
                                     &pitch_rate_p_output,
                                     &pitch_rate_i_output,
                                     &pitch_rate_d_output);
  yaw_output = motor_rate_pid_step(&yaw_rate_pid,
                                    motor_target_yaw_rate_dps,
                                    sensor_gyro_z_dps,
                                    NULL,
                                    NULL,
                                    NULL,
                                    NULL);

  motor_roll_output = roll_output;
  motor_pitch_output = pitch_output;
  motor_yaw_output = yaw_output;
  motor_effective_roll_rate_dps = effective_roll_rate_dps;
  motor_effective_pitch_rate_dps = effective_pitch_rate_dps;
  motor_roll_angle_error_deg = motor_target_roll_angle_deg - sensor_roll_deg;
  motor_roll_angle_output_dps = roll_angle_output_dps;
  motor_roll_rate_error_dps = roll_rate_error_dps;
  motor_roll_rate_p_output = roll_rate_p_output;
  motor_roll_rate_i_output = roll_rate_i_output;
  motor_roll_rate_d_output = roll_rate_d_output;
  motor_pitch_angle_error_deg = motor_target_pitch_angle_deg - sensor_pitch_deg;
  motor_pitch_angle_output_dps = pitch_angle_output_dps;
  motor_pitch_rate_error_dps = pitch_rate_error_dps;
  motor_pitch_rate_p_output = pitch_rate_p_output;
  motor_pitch_rate_i_output = pitch_rate_i_output;
  motor_pitch_rate_d_output = pitch_rate_d_output;

  motor_set_channels(motor_clamp_compare(base_compare - pitch_output + roll_output - yaw_output),
                     motor_clamp_compare(base_compare - pitch_output - roll_output + yaw_output),
                     motor_clamp_compare(base_compare + pitch_output - roll_output - yaw_output),
                     motor_clamp_compare(base_compare + pitch_output + roll_output + yaw_output));
}

//(1000.0f + ((float)base_compare - 1000.0f) * 1.02f)


void motor_stop(void)
{
  motor_set_throttle(MOTOR_OUTPUT_MIN_COMPARE);
  motor_reset_rate_pid();
  motor_set_all(MOTOR_OUTPUT_MIN_COMPARE);
}
