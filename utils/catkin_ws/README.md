### ROS workspace

This workspace contains useful ros nodes to drive Otto and get the odometry.

ROS packages and nodes:

* jopad_bridge:
    * joy_to_cmd_vel: used to translate joy msgs to cmd_vel msgs
    
* serial_bridge:
    * serial_transmitter: subscribes to cmd_vel_throttled and send the velocities setpoints to the board through the serial adapter.
    * serial_receiver: receives otto_status and publish odometry and tf.

ROS launches:
* serial\_bridge.launch: launches serial_bridge nodes.
* joypad.launch: launches the necessary nodes to drive Otto with a joypad. Requires joy, joy\_to\_cmd\_vel, serial_bridge.launch
* dario.launch: launches rosserial node needed for Dario's joypad. Thanks Dario.