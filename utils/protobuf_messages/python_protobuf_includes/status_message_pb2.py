# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: status_message.proto

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
  name='status_message.proto',
  package='',
  syntax='proto3',
  serialized_pb=_b('\n\x14status_message.proto\"\xc6\x01\n\x0fVelocityCommand\x12\x17\n\x0flinear_velocity\x18\x01 \x01(\x02\x12\x18\n\x10\x61ngular_velocity\x18\x02 \x01(\x02\x12\x14\n\x0c\x64\x65lta_millis\x18\x03 \x01(\x06\x12\'\n\x06status\x18\x04 \x01(\x0e\x32\x17.VelocityCommand.Status\"A\n\x06Status\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x12\n\x0eWAITING_CONFIG\x10\x01\x12\t\n\x05READY\x10\x02\x12\x0b\n\x07RUNNING\x10\x03\x62\x06proto3')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_VELOCITYCOMMAND_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='VelocityCommand.Status',
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
  ],
  containing_type=None,
  options=None,
  serialized_start=158,
  serialized_end=223,
)
_sym_db.RegisterEnumDescriptor(_VELOCITYCOMMAND_STATUS)


_VELOCITYCOMMAND = _descriptor.Descriptor(
  name='VelocityCommand',
  full_name='VelocityCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='linear_velocity', full_name='VelocityCommand.linear_velocity', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='angular_velocity', full_name='VelocityCommand.angular_velocity', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='delta_millis', full_name='VelocityCommand.delta_millis', index=2,
      number=3, type=6, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='VelocityCommand.status', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _VELOCITYCOMMAND_STATUS,
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=25,
  serialized_end=223,
)

_VELOCITYCOMMAND.fields_by_name['status'].enum_type = _VELOCITYCOMMAND_STATUS
_VELOCITYCOMMAND_STATUS.containing_type = _VELOCITYCOMMAND
DESCRIPTOR.message_types_by_name['VelocityCommand'] = _VELOCITYCOMMAND

VelocityCommand = _reflection.GeneratedProtocolMessageType('VelocityCommand', (_message.Message,), dict(
  DESCRIPTOR = _VELOCITYCOMMAND,
  __module__ = 'status_message_pb2'
  # @@protoc_insertion_point(class_scope:VelocityCommand)
  ))
_sym_db.RegisterMessage(VelocityCommand)


# @@protoc_insertion_point(module_scope)
