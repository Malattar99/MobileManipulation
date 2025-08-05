import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_angle_publisher')
        
        # Publishers for arm and base
        self.arm_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.base_pub = self.create_publisher(JointTrajectory, '/base_trajectory_controller/joint_trajectory', 10)
        
        # Timer for publishing trajectories
        self.timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_trajectories)
        
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_log_time = 0
        
    def publish_trajectories(self):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        t = t_now - self.start_time
        
        # Create messages
        arm_msg = JointTrajectory()
        arm_msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        base_msg = JointTrajectory()
        base_msg.joint_names = [
            'world_to_base_translate_x',
            'world_to_base_translate_y',
            'world_to_base_rotate_z'
        ]
        
        # Arm sine wave motion
        arm_point = JointTrajectoryPoint()
        amplitude = 0.3
        frequency = 0.5
        
        # Use modulo to keep the phase bounded
        phase = (2 * math.pi * frequency * t) % (2 * math.pi)
        arm_point.positions = [
            amplitude * math.sin(phase + i * math.pi / 6) - 0.4
            for i in range(6)
        ]
        
        arm_point.time_from_start.sec = int(self.timer_period)
        arm_point.time_from_start.nanosec = int((self.timer_period % 1.0) * 1e9)
        arm_msg.points.append(arm_point)
        
        # Base circular motion
        r = 0.5  # radius
        omega = 0.4  # angular velocity
        
        # Keep angle bounded between 0 and 2*pi
        angle = (omega * t) % (2 * math.pi)
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        
        # Keep yaw bounded between -pi and pi
        yaw = angle
        if yaw > math.pi:
            yaw -= 2 * math.pi
        
        base_point = JointTrajectoryPoint()
        base_point.positions = [x, y, yaw]
        
        lookahead = 0.2  # seconds into the future
        base_point.time_from_start.sec = int(lookahead)
        base_point.time_from_start.nanosec = int((lookahead % 1.0) * 1e9)
        base_msg.points.append(base_point)
        
        # Publish messages
        self.arm_pub.publish(arm_msg)
        self.base_pub.publish(base_msg)
        
        # Log every 5 seconds using a more robust method
        if t - self.last_log_time >= 5.0:
            self.get_logger().info(f'Running - t = {t:.2f}s | base: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
            self.last_log_time = t

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()