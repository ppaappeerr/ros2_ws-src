# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cartographer_ros_msgs:srv/StartTrajectory.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StartTrajectory_Request(type):
    """Metaclass of message 'StartTrajectory_Request'."""

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
            module = import_type_support('cartographer_ros_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cartographer_ros_msgs.srv.StartTrajectory_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_trajectory__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_trajectory__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_trajectory__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_trajectory__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_trajectory__request

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StartTrajectory_Request(metaclass=Metaclass_StartTrajectory_Request):
    """Message class 'StartTrajectory_Request'."""

    __slots__ = [
        '_configuration_directory',
        '_configuration_basename',
        '_use_initial_pose',
        '_initial_pose',
        '_relative_to_trajectory_id',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'configuration_directory': 'string',
        'configuration_basename': 'string',
        'use_initial_pose': 'boolean',
        'initial_pose': 'geometry_msgs/Pose',
        'relative_to_trajectory_id': 'int32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
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
        self.configuration_directory = kwargs.get('configuration_directory', str())
        self.configuration_basename = kwargs.get('configuration_basename', str())
        self.use_initial_pose = kwargs.get('use_initial_pose', bool())
        from geometry_msgs.msg import Pose
        self.initial_pose = kwargs.get('initial_pose', Pose())
        self.relative_to_trajectory_id = kwargs.get('relative_to_trajectory_id', int())

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
        if self.configuration_directory != other.configuration_directory:
            return False
        if self.configuration_basename != other.configuration_basename:
            return False
        if self.use_initial_pose != other.use_initial_pose:
            return False
        if self.initial_pose != other.initial_pose:
            return False
        if self.relative_to_trajectory_id != other.relative_to_trajectory_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def configuration_directory(self):
        """Message field 'configuration_directory'."""
        return self._configuration_directory

    @configuration_directory.setter
    def configuration_directory(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'configuration_directory' field must be of type 'str'"
        self._configuration_directory = value

    @builtins.property
    def configuration_basename(self):
        """Message field 'configuration_basename'."""
        return self._configuration_basename

    @configuration_basename.setter
    def configuration_basename(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'configuration_basename' field must be of type 'str'"
        self._configuration_basename = value

    @builtins.property
    def use_initial_pose(self):
        """Message field 'use_initial_pose'."""
        return self._use_initial_pose

    @use_initial_pose.setter
    def use_initial_pose(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'use_initial_pose' field must be of type 'bool'"
        self._use_initial_pose = value

    @builtins.property
    def initial_pose(self):
        """Message field 'initial_pose'."""
        return self._initial_pose

    @initial_pose.setter
    def initial_pose(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 'initial_pose' field must be a sub message of type 'Pose'"
        self._initial_pose = value

    @builtins.property
    def relative_to_trajectory_id(self):
        """Message field 'relative_to_trajectory_id'."""
        return self._relative_to_trajectory_id

    @relative_to_trajectory_id.setter
    def relative_to_trajectory_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'relative_to_trajectory_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'relative_to_trajectory_id' field must be an integer in [-2147483648, 2147483647]"
        self._relative_to_trajectory_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_StartTrajectory_Response(type):
    """Metaclass of message 'StartTrajectory_Response'."""

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
            module = import_type_support('cartographer_ros_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cartographer_ros_msgs.srv.StartTrajectory_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_trajectory__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_trajectory__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_trajectory__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_trajectory__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_trajectory__response

            from cartographer_ros_msgs.msg import StatusResponse
            if StatusResponse.__class__._TYPE_SUPPORT is None:
                StatusResponse.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StartTrajectory_Response(metaclass=Metaclass_StartTrajectory_Response):
    """Message class 'StartTrajectory_Response'."""

    __slots__ = [
        '_status',
        '_trajectory_id',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'status': 'cartographer_ros_msgs/StatusResponse',
        'trajectory_id': 'int32',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['cartographer_ros_msgs', 'msg'], 'StatusResponse'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
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
        from cartographer_ros_msgs.msg import StatusResponse
        self.status = kwargs.get('status', StatusResponse())
        self.trajectory_id = kwargs.get('trajectory_id', int())

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
        if self.status != other.status:
            return False
        if self.trajectory_id != other.trajectory_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if self._check_fields:
            from cartographer_ros_msgs.msg import StatusResponse
            assert \
                isinstance(value, StatusResponse), \
                "The 'status' field must be a sub message of type 'StatusResponse'"
        self._status = value

    @builtins.property
    def trajectory_id(self):
        """Message field 'trajectory_id'."""
        return self._trajectory_id

    @trajectory_id.setter
    def trajectory_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'trajectory_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'trajectory_id' field must be an integer in [-2147483648, 2147483647]"
        self._trajectory_id = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_StartTrajectory_Event(type):
    """Metaclass of message 'StartTrajectory_Event'."""

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
            module = import_type_support('cartographer_ros_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cartographer_ros_msgs.srv.StartTrajectory_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_trajectory__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_trajectory__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_trajectory__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_trajectory__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_trajectory__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StartTrajectory_Event(metaclass=Metaclass_StartTrajectory_Event):
    """Message class 'StartTrajectory_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<cartographer_ros_msgs/StartTrajectory_Request, 1>',
        'response': 'sequence<cartographer_ros_msgs/StartTrajectory_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['cartographer_ros_msgs', 'srv'], 'StartTrajectory_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['cartographer_ros_msgs', 'srv'], 'StartTrajectory_Response'), 1),  # noqa: E501
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
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

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
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from cartographer_ros_msgs.srv import StartTrajectory_Request
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, StartTrajectory_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'StartTrajectory_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from cartographer_ros_msgs.srv import StartTrajectory_Response
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, StartTrajectory_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'StartTrajectory_Response'"
        self._response = value


class Metaclass_StartTrajectory(type):
    """Metaclass of service 'StartTrajectory'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('cartographer_ros_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cartographer_ros_msgs.srv.StartTrajectory')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__start_trajectory

            from cartographer_ros_msgs.srv import _start_trajectory
            if _start_trajectory.Metaclass_StartTrajectory_Request._TYPE_SUPPORT is None:
                _start_trajectory.Metaclass_StartTrajectory_Request.__import_type_support__()
            if _start_trajectory.Metaclass_StartTrajectory_Response._TYPE_SUPPORT is None:
                _start_trajectory.Metaclass_StartTrajectory_Response.__import_type_support__()
            if _start_trajectory.Metaclass_StartTrajectory_Event._TYPE_SUPPORT is None:
                _start_trajectory.Metaclass_StartTrajectory_Event.__import_type_support__()


class StartTrajectory(metaclass=Metaclass_StartTrajectory):
    from cartographer_ros_msgs.srv._start_trajectory import StartTrajectory_Request as Request
    from cartographer_ros_msgs.srv._start_trajectory import StartTrajectory_Response as Response
    from cartographer_ros_msgs.srv._start_trajectory import StartTrajectory_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
