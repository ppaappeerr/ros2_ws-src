# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cpp_package:msg/ObstacleFeedback.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ObstacleFeedback(type):
    """Metaclass of message 'ObstacleFeedback'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('cpp_package')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cpp_package.msg.ObstacleFeedback')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__obstacle_feedback
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__obstacle_feedback
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__obstacle_feedback
            cls._TYPE_SUPPORT = module.type_support_msg__msg__obstacle_feedback
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__obstacle_feedback

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ObstacleFeedback(metaclass=Metaclass_ObstacleFeedback):
    """Message class 'ObstacleFeedback'."""

    __slots__ = [
        '_header',
        '_min_left',
        '_min_center',
        '_min_right',
        '_level_left',
        '_level_center',
        '_level_right',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'min_left': 'float',
        'min_center': 'float',
        'min_right': 'float',
        'level_left': 'uint8',
        'level_center': 'uint8',
        'level_right': 'uint8',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.min_left = kwargs.get('min_left', float())
        self.min_center = kwargs.get('min_center', float())
        self.min_right = kwargs.get('min_right', float())
        self.level_left = kwargs.get('level_left', int())
        self.level_center = kwargs.get('level_center', int())
        self.level_right = kwargs.get('level_right', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.min_left != other.min_left:
            return False
        if self.min_center != other.min_center:
            return False
        if self.min_right != other.min_right:
            return False
        if self.level_left != other.level_left:
            return False
        if self.level_center != other.level_center:
            return False
        if self.level_right != other.level_right:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if self._check_fields:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def min_left(self):
        """Message field 'min_left'."""
        return self._min_left

    @min_left.setter
    def min_left(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'min_left' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'min_left' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._min_left = value

    @builtins.property
    def min_center(self):
        """Message field 'min_center'."""
        return self._min_center

    @min_center.setter
    def min_center(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'min_center' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'min_center' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._min_center = value

    @builtins.property
    def min_right(self):
        """Message field 'min_right'."""
        return self._min_right

    @min_right.setter
    def min_right(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'min_right' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'min_right' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._min_right = value

    @builtins.property
    def level_left(self):
        """Message field 'level_left'."""
        return self._level_left

    @level_left.setter
    def level_left(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'level_left' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'level_left' field must be an unsigned integer in [0, 255]"
        self._level_left = value

    @builtins.property
    def level_center(self):
        """Message field 'level_center'."""
        return self._level_center

    @level_center.setter
    def level_center(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'level_center' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'level_center' field must be an unsigned integer in [0, 255]"
        self._level_center = value

    @builtins.property
    def level_right(self):
        """Message field 'level_right'."""
        return self._level_right

    @level_right.setter
    def level_right(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'level_right' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'level_right' field must be an unsigned integer in [0, 255]"
        self._level_right = value
