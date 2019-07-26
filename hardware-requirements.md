## Hardware requirements and peripherals

### Motor drivers
[High-power motor drivers](https://www.pololu.com/category/11/brushed-dc-motor-drivers)

We have to control 2 motors so a 2 channel driver should be the best choice.
TBD which one depending on the motor current needs.

### Encoders

TBD

### PIN requirements
##### pin needed for encoders:
2 GPIO for each encoder, 4 GPIO total

##### pin needed for drivers:
* 6 GPIO for fault indicators, sleep inputs, motor directions
* 2 PWM pins for motor speed control
* 2 Analog pins for current sense

##### Total pin count:
* 10 GPIO
* 2 PWM
* 2 Analog
