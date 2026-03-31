#include "switch.h"

volatile uint8_t sw_u_flag = 0U;
volatile uint8_t sw_d_flag = 0U;
volatile uint8_t sw_l_flag = 0U;
volatile uint8_t sw_r_flag = 0U;
volatile uint8_t sw_p_flag = 0U;

static GPIO_PinState sw_u_prev_state = GPIO_PIN_SET;
static GPIO_PinState sw_d_prev_state = GPIO_PIN_SET;
static GPIO_PinState sw_l_prev_state = GPIO_PIN_SET;
static GPIO_PinState sw_r_prev_state = GPIO_PIN_SET;
static GPIO_PinState sw_p_prev_state = GPIO_PIN_SET;

void switch_init(void)
{
  sw_u_prev_state = HAL_GPIO_ReadPin(SW_U_GPIO_Port, SW_U_Pin);
  sw_d_prev_state = HAL_GPIO_ReadPin(SW_D_GPIO_Port, SW_D_Pin);
  sw_l_prev_state = HAL_GPIO_ReadPin(SW_L_GPIO_Port, SW_L_Pin);
  sw_r_prev_state = HAL_GPIO_ReadPin(SW_R_GPIO_Port, SW_R_Pin);
  sw_p_prev_state = HAL_GPIO_ReadPin(SW_P_GPIO_Port, SW_P_Pin);

  sw_u_flag = 0U;
  sw_d_flag = 0U;
  sw_l_flag = 0U;
  sw_r_flag = 0U;
  sw_p_flag = 0U;
}

void switch_update(void)
{
  GPIO_PinState sw_u_state = HAL_GPIO_ReadPin(SW_U_GPIO_Port, SW_U_Pin);
  GPIO_PinState sw_d_state = HAL_GPIO_ReadPin(SW_D_GPIO_Port, SW_D_Pin);
  GPIO_PinState sw_l_state = HAL_GPIO_ReadPin(SW_L_GPIO_Port, SW_L_Pin);
  GPIO_PinState sw_r_state = HAL_GPIO_ReadPin(SW_R_GPIO_Port, SW_R_Pin);
  GPIO_PinState sw_p_state = HAL_GPIO_ReadPin(SW_P_GPIO_Port, SW_P_Pin);

  sw_u_flag = 0U;
  sw_d_flag = 0U;
  sw_l_flag = 0U;
  sw_r_flag = 0U;
  sw_p_flag = 0U;

  if ((sw_u_prev_state == GPIO_PIN_SET) && (sw_u_state == GPIO_PIN_RESET))
  {
    sw_u_flag = 1U;
  }

  if ((sw_d_prev_state == GPIO_PIN_SET) && (sw_d_state == GPIO_PIN_RESET))
  {
    sw_d_flag = 1U;
  }

  if ((sw_l_prev_state == GPIO_PIN_SET) && (sw_l_state == GPIO_PIN_RESET))
  {
    sw_l_flag = 1U;
  }

  if ((sw_r_prev_state == GPIO_PIN_SET) && (sw_r_state == GPIO_PIN_RESET))
  {
    sw_r_flag = 1U;
  }

  if ((sw_p_prev_state == GPIO_PIN_SET) && (sw_p_state == GPIO_PIN_RESET))
  {
    sw_p_flag = 1U;
  }

  sw_u_prev_state = sw_u_state;
  sw_d_prev_state = sw_d_state;
  sw_l_prev_state = sw_l_state;
  sw_r_prev_state = sw_r_state;
  sw_p_prev_state = sw_p_state;
}
