#ifndef __SWITCH_H__
#define __SWITCH_H__

#include "main.h"

extern volatile uint8_t sw_u_flag;
extern volatile uint8_t sw_d_flag;
extern volatile uint8_t sw_l_flag;
extern volatile uint8_t sw_r_flag;
extern volatile uint8_t sw_p_flag;

void switch_init(void);
void switch_update(void);

#endif
