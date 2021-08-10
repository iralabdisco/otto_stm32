# Otto

Firmware for Otto's controlboard. Otto is a differential drive robot.

The main program is located in [otto_controller](https://github.com/iralabdisco/otto/tree/master/otto_controller).

## Instructions

If you just want to use the robot you can download the binary file from the [releases](https://github.com/iralabdisco/otto/releases): 

1. Download the lastest otto_controller.bin
2. Plug your Nucleo boart to the PC.
3. Copy and paste the .bin file inside your board.

## Coding a driver for otto

You will need to have a way to open a serial port on your PC.

1. Before doing anything reset the MCU using the DTR pin in the serial port.
2. The MCU will then wait to receive a ConfigMessage.
3. After the config message is received the PID loop control starts.
4. Every time a VelocityMessage is received Otto will send a StatusMessage.

Refer to [otto_messages.h](https://github.com/iralabdisco/otto_stm32/blob/master/otto_controller/Core/Inc/communication/otto_messages.h) to see how the messages are structured.

The CRC standard used is CRC-32/MPEG-2, tested with python package [crccheck](https://pythonhosted.org/crccheck/crccheck.html)

## Coding style

Follow [Google C++ guideline](https://google.github.io/styleguide/cppguide.html) while working on the project

Eclipse's formatting configuration file can be found [here](https://github.com/google/styleguide/blob/gh-pages/eclipse-cpp-google-style.xml)

