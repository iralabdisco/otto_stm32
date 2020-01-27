### ROS workspace

This workspace contains useful ros nodes to drive Otto and get the odometry.

ROS nodes:

* joy_to_cmd_vel: used to translate joy msgs to cmd_vel msgs
* cmd_vel_transmitter: used to send the velocities setpoints to the board through the serial adapter.

ROS launches:
* joypad.launch: launches the necessary nodes to drive Otto with a joypad. Requires joy, joy_to_cmd_vel, cmd_vel_transmitter.
