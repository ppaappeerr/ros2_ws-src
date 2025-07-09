// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from cpp_package:msg/ObstacleFeedback.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "cpp_package/msg/detail/obstacle_feedback__struct.h"
#include "cpp_package/msg/detail/obstacle_feedback__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool cpp_package__msg__obstacle_feedback__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[52];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("cpp_package.msg._obstacle_feedback.ObstacleFeedback", full_classname_dest, 51) == 0);
  }
  cpp_package__msg__ObstacleFeedback * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // min_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_left");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_left = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // min_center
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_center");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_center = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // min_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_right");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_right = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // level_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "level_left");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->level_left = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // level_center
    PyObject * field = PyObject_GetAttrString(_pymsg, "level_center");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->level_center = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // level_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "level_right");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->level_right = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * cpp_package__msg__obstacle_feedback__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ObstacleFeedback */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("cpp_package.msg._obstacle_feedback");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ObstacleFeedback");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  cpp_package__msg__ObstacleFeedback * ros_message = (cpp_package__msg__ObstacleFeedback *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_left
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_center
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_center);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_center", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_right
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // level_left
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->level_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "level_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // level_center
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->level_center);
    {
      int rc = PyObject_SetAttrString(_pymessage, "level_center", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // level_right
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->level_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "level_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
