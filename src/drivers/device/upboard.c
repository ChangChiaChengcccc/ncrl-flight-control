#include <stdint.h>
#include <string.h>
#include "uart.h"
#include "imu.h"
#include "gpio.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "stm32f4xx_conf.h"
#include "task.h"
#include "sys_time.h"
#include "debug_link.h"
#include "upboard.h"
#include "proj_config.h"
#include "multirotor_geometry_ctrl.h"
#include "sbus_radio.h"

#define UPBOARD_QUEUE_SIZE (44 * 100) //~400 packets

#define STM32_TO_UPBOARD_CHECKSUM_INIT_VAL 0

float receive_e1 = 1.0f;
float receive_e2 = 1.0f;
float receive_e3 = 1.0f;
float receive_e4 = 1.0f;

static uint8_t generate_stm32_to_upboard_checksum_byte(uint8_t *payload, int payload_cnt)
{
	uint8_t result = STM32_TO_UPBOARD_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_cnt; i++)
		result ^= payload[i];

	return result;
}

void send_stm32_msg(void)
{	
	/* extern variable name
	float ukf_pos_enu[3] = {0};
	float ukf_vel_enu[3] = {0};
	float ukf_acc_enu[3] = {0}; 
	float ukf_W[3] = {0};
	float ukf_f1_cmd = 0;
	float ukf_f2_cmd = 0;
	float ukf_f3_cmd = 0;
	float ukf_f4_cmd = 0;
	float ukf_RotMat_array[9] = {0};
	*/
	/*----------------------------------------------------------------------------------
	 *| start byte | checksum | pos[x,y,z] | W[x,y,z] | f[1~4]_cmd | R[0~8] | end byte |
	 *----------------------------------------------------------------------------------*/


	char msg_buf[STM32_TO_UPBOARD_MSG_SIZE] = {0};
	int msg_pos = 0;

	/* reserve 2 for start byte and checksum byte as header */
	msg_buf[msg_pos] = '@'; //start byte
	msg_pos += sizeof(uint8_t);
	msg_buf[msg_pos] = 0;	//checksum
	msg_pos += sizeof(uint8_t);
	//float fuck[3] = {1,2,3};
	/* pack payloads */
	memcpy(msg_buf + msg_pos, &ukf_pos_enu[0], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_pos_enu[1], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_pos_enu[2], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_W[0], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_W[1], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_W[2], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_f1_cmd, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_f2_cmd, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_f3_cmd, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_f4_cmd, sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_RotMat_array[0], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_RotMat_array[1], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_RotMat_array[2], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_RotMat_array[3], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_RotMat_array[4], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_RotMat_array[5], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_RotMat_array[6], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_RotMat_array[7], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_RotMat_array[8], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_vel_enu[0], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_vel_enu[1], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_vel_enu[2], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_acc_enu[0], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_acc_enu[1], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &ukf_acc_enu[2], sizeof(float));
	msg_pos += sizeof(float);


	msg_buf[msg_pos] = '+'; //end byte
	msg_pos += sizeof(uint8_t);

	msg_buf[1] = generate_stm32_to_upboard_checksum_byte((uint8_t *)&msg_buf[2],
	                STM32_TO_UPBOARD_MSG_SIZE - 3);

	uart6_puts(msg_buf, STM32_TO_UPBOARD_MSG_SIZE);
}

void stm32_send_msg_20hz(void)
{
	/* triggered every 20 times since the function is designed to be called by
	 * flight control main loop (400Hz) */
	static int prescaler = 10;
	prescaler--;

	if(prescaler == 0) {
		send_stm32_msg();
		prescaler = 10;
	}
}



//receive upboard info

typedef struct {
	char c;
} upboard_buf_c_t;

QueueHandle_t upboard_queue;

upboard_t upboard;

void upboard_init(int id)
{
	upboard.id = id;
	upboard_queue = xQueueCreate(UPBOARD_QUEUE_SIZE, sizeof(upboard_buf_c_t));
}

bool upboard_available(void)
{
	//timeout if no data available more than 300ms
	float current_time = get_sys_time_ms();
	if((current_time - upboard.time_now) > 300) {
		return false;
	}
	return true;
}

void upboard_buf_push(uint8_t c)
{
	if(upboard.buf_pos >= UPBOARD_TO_STM32_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < UPBOARD_TO_STM32_MSG_SIZE; i++) {
			upboard.buf[i - 1] = upboard.buf[i];
		}

		/* save new byte to the last array element */
		upboard.buf[UPBOARD_TO_STM32_MSG_SIZE - 1] = c;
		upboard.buf_pos = UPBOARD_TO_STM32_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		upboard.buf[upboard.buf_pos] = c;
		upboard.buf_pos++;
	}
}

void upboard_isr_handler(uint8_t c)
{
	upboard_buf_c_t upboard_queue_item;
	upboard_queue_item.c = c;

	BaseType_t higher_priority_task_woken = pdFALSE;
	xQueueSendToBackFromISR(upboard_queue, &upboard_queue_item,
	                        &higher_priority_task_woken);
	portEND_SWITCHING_ISR(higher_priority_task_woken);
}

int upboard_serial_decoder(uint8_t *buf)
{
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_stm32_to_upboard_checksum_byte(&buf[2], UPBOARD_TO_STM32_MSG_SIZE - 3);
	
	if(checksum != recv_checksum) {
		return 1; //error detected
		//printf("garbage message\n");
	}

	upboard.time_now = get_sys_time_ms();

	//float receive efficiency
	memcpy(&receive_e1, &buf[2], sizeof(float));
	memcpy(&receive_e2, &buf[6], sizeof(float));
	memcpy(&receive_e3, &buf[10], sizeof(float));
	memcpy(&receive_e4, &buf[14], sizeof(float));

#if (UAV_DEFAULT_ID == 1)
	upboard.ukf_efficiency[0] = receive_e1;
	upboard.ukf_efficiency[1] = receive_e2;
	upboard.ukf_efficiency[2] = receive_e3;
	upboard.ukf_efficiency[3] = receive_e4;

#elif (UAV_DEFAULT_ID == 2)
	upboard.ukf_force[0] = receive_force_x;
	upboard.ukf_force[1] = receive_force_y;
	upboard.ukf_force[2] = receive_force_z;
	memcpy(&receive_payload_yaw, &buf[15], sizeof(float));
	upboard.yaw = receive_payload_yaw;
#endif

	upboard.time_last = upboard.time_now;
	return 0;
}



void upboard_update(void)
{
	upboard_buf_c_t recept_c;
	while(xQueueReceive(upboard_queue, &recept_c, 0) == pdTRUE) {
		uint8_t c = recept_c.c;
		upboard_buf_push(c);
		if(c == '+' && upboard.buf[0] == '@') {
			/* decode upboard message */
			if(upboard_serial_decoder(upboard.buf) == 0) {
				upboard.buf_pos = 0; //reset position pointer
			}
		}

	}
}


void get_upboard_ukf_e1(float *e1)
{
	*e1 = upboard.ukf_efficiency[0];
}

void get_upboard_ukf_e2(float *e2)
{
	*e2 = upboard.ukf_efficiency[1];
}

void get_upboard_ukf_e3(float *e3)
{
	*e3 = upboard.ukf_efficiency[2];
}

void get_upboard_ukf_e4(float *e4)
{
	*e4 = upboard.ukf_efficiency[3];
}
