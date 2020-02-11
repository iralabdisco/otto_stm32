# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: otto_communication.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='otto_communication.proto',
  package='',
  syntax='proto2',
  serialized_pb=_b('\n\x18otto_communication.proto\"\x82\x02\n\rStatusMessage\x12\x17\n\x0flinear_velocity\x18\x01 \x02(\x02\x12\x18\n\x10\x61ngular_velocity\x18\x02 \x02(\x02\x12\x14\n\x0c\x64\x65lta_millis\x18\x03 \x02(\x06\x12%\n\x06status\x18\x04 \x02(\x0e\x32\x15.StatusMessage.Status\"\x80\x01\n\x06Status\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x12\n\x0eWAITING_CONFIG\x10\x01\x12\t\n\x05READY\x10\x02\x12\x0b\n\x07RUNNING\x10\x03\x12\x14\n\x10H_BRIDGE_FAULT_1\x10\x04\x12\x14\n\x10H_BRIDGE_FAULT_2\x10\x05\x12\x11\n\rUNKNOWN_ERROR\x10\x06\"\xa3\x02\n\rConfigCommand\x12\x0f\n\x07left_kp\x18\x01 \x02(\x02\x12\x0f\n\x07left_ki\x18\x02 \x02(\x02\x12\x0f\n\x07left_kd\x18\x03 \x02(\x02\x12\x10\n\x08right_kp\x18\x04 \x02(\x02\x12\x10\n\x08right_ki\x18\x05 \x02(\x02\x12\x10\n\x08right_kd\x18\x06 \x02(\x02\x12\x10\n\x08\x63ross_kp\x18\x07 \x02(\x02\x12\x10\n\x08\x63ross_ki\x18\x08 \x02(\x02\x12\x10\n\x08\x63ross_kd\x18\t \x02(\x02\x12\x10\n\x08\x62\x61seline\x18\n \x02(\x02\x12\x1c\n\x14ticks_per_revolution\x18\x0b \x02(\x07\x12!\n\x19right_wheel_circumference\x18\x0c \x02(\x02\x12 \n\x18left_wheel_circumference\x18\r \x02(\x02\"D\n\x0fVelocityCommand\x12\x17\n\x0flinear_velocity\x18\x01 \x02(\x02\x12\x18\n\x10\x61ngular_velocity\x18\x02 \x02(\x02')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_STATUSMESSAGE_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='StatusMessage.Status',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WAITING_CONFIG', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='READY', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RUNNING', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='H_BRIDGE_FAULT_1', index=4, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='H_BRIDGE_FAULT_2', index=5, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_ERROR', index=6, number=6,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=159,
  serialized_end=287,
)
_sym_db.RegisterEnumDescriptor(_STATUSMESSAGE_STATUS)


_STATUSMESSAGE = _descriptor.Descriptor(
  name='StatusMessage',
  full_name='StatusMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='linear_velocity', full_name='StatusMessage.linear_velocity', index=0,
      number=1, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='angular_velocity', full_name='StatusMessage.angular_velocity', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='delta_millis', full_name='StatusMessage.delta_millis', index=2,
      number=3, type=6, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='StatusMessage.status', index=3,
      number=4, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _STATUSMESSAGE_STATUS,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=29,
  serialized_end=287,
)


_CONFIGCOMMAND = _descriptor.Descriptor(
  name='ConfigCommand',
  full_name='ConfigCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='left_kp', full_name='ConfigCommand.left_kp', index=0,
      number=1, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='left_ki', full_name='ConfigCommand.left_ki', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='left_kd', full_name='ConfigCommand.left_kd', index=2,
      number=3, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_kp', full_name='ConfigCommand.right_kp', index=3,
      number=4, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_ki', full_name='ConfigCommand.right_ki', index=4,
      number=5, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_kd', full_name='ConfigCommand.right_kd', index=5,
      number=6, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cross_kp', full_name='ConfigCommand.cross_kp', index=6,
      number=7, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cross_ki', full_name='ConfigCommand.cross_ki', index=7,
      number=8, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cross_kd', full_name='ConfigCommand.cross_kd', index=8,
      number=9, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='baseline', full_name='ConfigCommand.baseline', index=9,
      number=10, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ticks_per_revolution', full_name='ConfigCommand.ticks_per_revolution', index=10,
      number=11, type=7, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_wheel_circumference', full_name='ConfigCommand.right_wheel_circumference', index=11,
      number=12, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='left_wheel_circumference', full_name='ConfigCommand.left_wheel_circumference', index=12,
      number=13, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=290,
  serialized_end=581,
)


_VELOCITYCOMMAND = _descriptor.Descriptor(
  name='VelocityCommand',
  full_name='VelocityCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='linear_velocity', full_name='VelocityCommand.linear_velocity', index=0,
      number=1, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='angular_velocity', full_name='VelocityCommand.angular_velocity', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=583,
  serialized_end=651,
)

_STATUSMESSAGE.fields_by_name['status'].enum_type = _STATUSMESSAGE_STATUS
_STATUSMESSAGE_STATUS.containing_type = _STATUSMESSAGE
DESCRIPTOR.message_types_by_name['StatusMessage'] = _STATUSMESSAGE
DESCRIPTOR.message_types_by_name['ConfigCommand'] = _CONFIGCOMMAND
DESCRIPTOR.message_types_by_name['VelocityCommand'] = _VELOCITYCOMMAND

StatusMessage = _reflection.GeneratedProtocolMessageType('StatusMessage', (_message.Message,), dict(
  DESCRIPTOR = _STATUSMESSAGE,
  __module__ = 'otto_communication_pb2'
  # @@protoc_insertion_point(class_scope:StatusMessage)
  ))
_sym_db.RegisterMessage(StatusMessage)

ConfigCommand = _reflection.GeneratedProtocolMessageType('ConfigCommand', (_message.Message,), dict(
  DESCRIPTOR = _CONFIGCOMMAND,
  __module__ = 'otto_communication_pb2'
  # @@protoc_insertion_point(class_scope:ConfigCommand)
  ))
_sym_db.RegisterMessage(ConfigCommand)

VelocityCommand = _reflection.GeneratedProtocolMessageType('VelocityCommand', (_message.Message,), dict(
  DESCRIPTOR = _VELOCITYCOMMAND,
  __module__ = 'otto_communication_pb2'
  # @@protoc_insertion_point(class_scope:VelocityCommand)
  ))
_sym_db.RegisterMessage(VelocityCommand)


# @@protoc_insertion_point(module_scope)
