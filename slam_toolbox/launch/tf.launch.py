# tf.launch.py
from launch_ros.actions import Node
...
static_tf_laser = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_laser',
    arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
)

static_tf_map = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_map',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
)

