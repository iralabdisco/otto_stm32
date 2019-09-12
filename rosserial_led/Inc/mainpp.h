#pragma once

#include <std_msgs/UInt8.h>
#include <ros.h>

void led_cb(const std_msgs::UInt8& msg);
void setup();
void loop();
