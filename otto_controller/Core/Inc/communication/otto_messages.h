/*
 * otto_messages.h
 *
 *  Created on: Aug 5, 2021
 *      Author: fdila
 */

#ifndef INC_COMMUNICATION_OTTO_MESSAGES_H_
#define INC_COMMUNICATION_OTTO_MESSAGES_H_

typedef struct _ConfigMessage {
  uint32_t ticks_per_revolution;  //x4 resolution
  float baseline;  //in meters
  float left_wheel_circumference;  //in meters
  float right_wheel_circumference;  //in meters
  float kp_left;
  float ki_left;
  float kd_left;
  float kp_right;
  float ki_right;
  float kd_right;
  float kp_cross;
  float ki_cross;
  float kd_cross;
  uint32_t crc;
} ConfigMessage;

typedef struct _VelocityMessage {
  float linear_velocity;
  float angular_velocity;
  uint32_t crc;
} VelocityMessage;

typedef struct _StatusMessage {
  /*
   * Status codes:
   * 0 - waiting for config
   * 1 - running
   * 2 - error receiving config
   * 3 - error receiving vel
   * 4 - H-Bridge fault
   */
  uint16_t status;
  uint16_t delta_millis;
  int32_t left_ticks;
  int32_t right_ticks;
  uint32_t crc;
} StatusMessage;

#endif /* INC_COMMUNICATION_OTTO_MESSAGES_H_ */
