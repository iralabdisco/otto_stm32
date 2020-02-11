import otto_communication_pb2

status_message = otto_communication_pb2.StatusMessage()

status_message.linear_velocity = 0.0
status_message.angular_velocity = 0.0
status_message.delta_millis = 0
status_message.status = otto_communication_pb2.StatusMessage.Status.UNKNOWN

encode_buffer = status_message.SerializeToString()
status_message_length = len(encode_buffer)
print(encode_buffer)
print(status_message_length)
status_message.ParseFromString(encode_buffer)
print(status_message)

my_velocity = otto_communication_pb2.VelocityCommand()
my_velocity.linear_velocity = 0.6
my_velocity.angular_velocity = -0.1

encode_buffer = my_velocity.SerializeToString()
print(encode_buffer)
print(len(encode_buffer))


my_config = otto_communication_pb2.ConfigCommand()
print(my_config)

