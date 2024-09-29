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

        # Parameters
        self.distance = 1.0  # Total distance in meters
        self.max_speed = 0.2  # Max speed 
        self.acceleration = 0.1  # Acceleration 
        self.deceleration = -0.1  # Deceleration

        # Distances for acceleration and deceleration
        self.accel_distance = (self.max_speed ** 2) / (2 * self.acceleration)
        self.decel_distance = (self.max_speed ** 2) / (2 * -self.deceleration)
        
        # Calculate constant velocity distance
        if self.accel_distance + self.decel_distance < self.distance:
            self.constant_velocity_distance = self.distance - (self.accel_distance + self.decel_distance)
        else:
            self.constant_velocity_distance = 0.0

        # Initial states
        self.current_speed = 0.0
        self.phase = 'accelerating'
        self.position = 0.0
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def move_turtlebot(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.current_speed
        self.pub.publish(twist_msg)

    def timer_callback(self):
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        self.position += self.current_speed * 0.1  # Update position

        # Record position and time
        self.x_positions.append(self.position)
        self.time_steps.append(current_time)

        # Update motion phase
        if self.phase == 'accelerating':
            if self.current_speed < self.max_speed:
                self.current_speed += self.acceleration * 0.1
                if self.current_speed >= self.max_speed:
                    self.current_speed = self.max_speed
                    self.phase = 'constant'  # Switch to constant speed
        elif self.phase == 'constant':
            if self.position < (self.accel_distance + self.constant_velocity_distance):
                self.current_speed = self.max_speed
            else:
                self.phase = 'decelerating'  # Switch to deceleration
        elif self.phase == 'decelerating':
            if self.current_speed > 0:
                self.current_speed += self.deceleration * 0.1
                if self.current_speed < 0:
                    self.current_speed = 0  # Stop
                    self.timer.cancel()  # Stop the timer
                    self.plot_trajectory()  # Plot trajectory

        # Publish current speed
        twist_msg = Twist()
        twist_msg.linear.x = self.current_speed
        self.pub.publish(twist_msg)

    def plot_trajectory(self):
        plt.plot(self.time_steps, self.x_positions, label='TurtleBot Trajectory')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Trajectory of TurtleBot in scenario 2')
        plt.grid()
        plt.axhline(y=self.distance, color='r', linestyle='--', label='Target Position')
        plt.legend()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    controller.move_turtlebot()  # Start moving the TurtleBot
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

