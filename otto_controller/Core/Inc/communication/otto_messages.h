/*
 * otto_messages.h
 *
 *  Created on: Aug 5, 2021
 *      Author: fdila
 */

#ifndef INC_COMMUNICATION_OTTO_MESSAGES_H_
#define INC_COMMUNICATION_OTTO_MESSAGES_H_

typedef struct _VelocityMessage {
    float linear_velocity;
    float angular_velocity;
    uint32_t crc;
} VelocityMessage;

typedef struct _StatusMessage {
	uint16_t status;
	uint16_t delta_millis;
    uint32_t left_ticks;
    uint32_t right_ticks;
    uint32_t crc;
} StatusMessage;


#endif /* INC_COMMUNICATION_OTTO_MESSAGES_H_ */
