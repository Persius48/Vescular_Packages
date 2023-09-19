import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ConstantVelocityPublisher(Node):
    def __init__(self):
        super().__init__('constant_velocity_publisher')
        
        self.declare_parameter('linear_velocity', 0.06)
        self.declare_parameter('angular_velocity', 0.0)
        self.declare_parameter('duration', 5.0)
        
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.duration = self.get_parameter('duration').value
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_time = 0.0
        
    def timer_callback(self):
        if self.current_time < self.duration:
            msg = Twist()
            msg.linear.x = self.linear_velocity
            msg.angular.z = self.angular_velocity
            self.publisher.publish(msg)
            
            self.current_time += 0.1  # 100ms timer interval
        else:
            self.get_logger().info("Time is up")

def main(args=None):
    rclpy.init(args=args)
    node = ConstantVelocityPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
