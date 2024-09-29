import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer 
        self.x_positions = []
        self.time_steps = []
        self.start_time = None
        self.distance = 1.0  # Target distance in meters
        self.time = 10.0  # Time to reach the target distance
        self.v = self.distance / self.time  # Constant velocity

        self.move_turtlebot()

    def move_turtlebot(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.v
        self.pub.publish(twist_msg)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        if self.start_time is not None:
            current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
            if current_time <= self.time:
                self.x_positions.append(self.v * current_time)
                self.time_steps.append(current_time)
            else:
                self.stop_turtlebot()
                self.timer.cancel()
                self.plot_trajectory()

    def stop_turtlebot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub.publish(twist_msg)

    def plot_trajectory(self):
        plt.plot(self.time_steps, self.x_positions, label='TurtleBot Trajectory')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Trajectory of Turtlebot Scenario 1')
        plt.grid()
        plt.axhline(y=self.distance, color='r', linestyle='--', label='Target Position')
        plt.legend()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

