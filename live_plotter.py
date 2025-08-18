import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading

# Deques to store data for plotting, with a maximum length
history_size = 100  # Number of data points to display
time_data = deque(maxlen=history_size)
angle_data = deque(maxlen=history_size)
velocity_data = deque(maxlen=history_size)

class LivePlotterNode(Node):
    """
    A ROS2 node that subscribes to plotting topics and updates data deques.
    The plotting itself is handled separately to avoid blocking ROS callbacks.
    """
    def __init__(self):
        super().__init__('live_plotter_subscriber_node')
        self.create_subscription(Float64, '/plot/path_angle', self.angle_callback, 10)
        self.create_subscription(Float64, '/plot/path_angular_velocity', self.velocity_callback, 10)
        self.start_time = self.get_clock().now()
        self.last_angle_time = None
        self.last_velocity_time = None

    def angle_callback(self, msg):
        if self.last_angle_time is None:
             self.last_angle_time = self.get_clock().now()
        
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        time_data.append(current_time)
        angle_data.append(msg.data)
        self.last_angle_time = self.get_clock().now()


    def velocity_callback(self, msg):
        # This assumes velocity is published at a similar rate to angle.
        # We use the time data from the angle callback for a shared x-axis.
        if self.last_velocity_time is None:
             self.last_velocity_time = self.get_clock().now()
        velocity_data.append(msg.data)
        self.last_velocity_time = self.get_clock().now()


def update_plot(frame, fig, ax1, ax2, line1, line2):
    """
    This function is called by FuncAnimation to redraw the plot.
    """
    # To handle cases where velocity data arrives slightly slower
    min_len = min(len(time_data), len(angle_data), len(velocity_data))
    
    if min_len > 0:
        times = list(time_data)[-min_len:]
        angles = list(angle_data)[-min_len:]
        velocities = list(velocity_data)[-min_len:]

        line1.set_data(times, angles)
        line2.set_data(times, velocities)

        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()

    return line1, line2

def ros_thread_func():
    """Function to run the ROS2 node in a separate thread."""
    rclpy.init()
    plotter_node = LivePlotterNode()
    rclpy.spin(plotter_node)
    plotter_node.destroy_node()
    rclpy.shutdown()

def main():
    """
    Main function to set up the plot and start the ROS thread.
    """
    # --- Set up the plot ---
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    fig.suptitle('Real-time Path Analysis')

    # Plot 1: Angle
    line1, = ax1.plot([], [], 'r-')
    ax1.set_ylabel('Path Angle (deg)')
    ax1.set_ylim(-190, 190)  # Set fixed Y-axis for angle (+-180 with margin)
    ax1.grid()

    # Plot 2: Angular Velocity
    line2, = ax2.plot([], [], 'b-')
    ax2.set_ylabel('Angular Velocity (deg/s)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylim(-400, 400) # Set a generous fixed Y-axis for velocity
    ax2.grid()

    # --- Run ROS node in a background thread ---
    ros_thread = threading.Thread(target=ros_thread_func, daemon=True)
    ros_thread.start()

    # --- Set up and start the animation ---
    ani = FuncAnimation(fig, update_plot, fargs=(fig, ax1, ax2, line1, line2),
                        interval=100, blit=True)
    
    plt.show()

if __name__ == '__main__':
    main()
