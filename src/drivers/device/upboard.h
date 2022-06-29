#ifndef __UPBOARD_INTERFACE_H__
#define __UPBOARD_INTERFACE_H__
#include "proj_config.h"

#define STM32_TO_UPBOARD_MSG_SIZE 103

#if (UAV_DEFAULT_ID == 1)
#define UPBOARD_TO_STM32_MSG_SIZE 19
//#pragma message("UPBOARD_TO_STM32_MSG_SIZE is 19")
#endif

#if (UAV_DEFAULT_ID == 2)
#define UPBOARD_TO_STM32_MSG_SIZE 20
#pragma message("UPBOARD_TO_STM32_MSG_SIZE is 20")
#endif

typedef struct {
	uint8_t id;

	/* force [N] */
	float ukf_efficiency[4];
	float time_now;
	float time_last;
	float update_rate;

	volatile int buf_pos;
	uint8_t buf[200];
} upboard_t ;

void upboard_init(int id);
void upboard_isr_handler(uint8_t c);

void get_upboard_ukf_e1(float *e1);
void get_upboard_ukf_e2(float *e2);
void get_upboard_ukf_e3(float *e3);
void get_upboard_ukf_e4(float *e4);

void upboard_update(void);

void send_stm32_msg(void);
void stm32_send_msg_20hz(void);

#endif