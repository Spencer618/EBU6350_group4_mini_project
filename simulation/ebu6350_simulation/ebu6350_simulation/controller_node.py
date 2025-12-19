import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile

class IncrementalPID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0.0
        self.last_error = 0.0
        self.prev_error = 0.0
        self.output = 0.0
        self.output_last = 0.0

    def update(self, error):
        self.error = error
        d_error = self.error - self.last_error
        dd_error = d_error - (self.last_error - self.prev_error)
        delta_output = (self.kp * d_error) + (self.ki * self.error) + (self.kd * dd_error)
        self.output = self.output_last + delta_output
        self.prev_error = self.last_error
        self.last_error = self.error
        self.output_last = self.output
        return self.output

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        qos = QoSProfile(depth=10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, qos)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.target_yaw = 0.0
        self.current_yaw = 0.0
        self.base_speed = 0.2
        self.state = 0
        self.front_distance = 9.9
        self.angle_pid = IncrementalPID(kp=1.5, ki=0.0, kd=0.1)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def scan_callback(self, msg):
        if len(msg.ranges) > 0:
            self.front_distance = msg.ranges[0]

    def control_loop(self):
        if self.front_distance < 0.5 and self.state == 0:
            self.get_logger().info("Obstacle detected! Initiating turn sequence.")
            self.state = 1
            target = self.current_yaw + 90
            if target > 180: target -= 360
            self.target_yaw = target

        error = self.target_yaw - self.current_yaw
        if error > 180: error -= 360
        elif error < -180: error += 360
        
        turn_output = self.angle_pid.update(error)
        twist = Twist()
        if abs(error) < 2.0:
            twist.linear.x = self.base_speed
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.05
            twist.angular.z = turn_output * 0.05
        self.pub_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
