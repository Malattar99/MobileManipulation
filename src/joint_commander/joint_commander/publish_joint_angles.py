#!/usr/bin/env python3

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

        # Subscribers for external trajectories from motion generator
        self.create_subscription(
            JointTrajectory,
            '/motion_library/arm_trajectory',
            self.arm_trajectory_callback,
            10
        )
        
        self.create_subscription(
            JointTrajectory,
            '/motion_library/base_trajectory',
            self.base_trajectory_callback,
            10
        )

        self.timer_period = 0.02  # 50 Hz 
        self.timer = self.create_timer(self.timer_period, self.publish_trajectories)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        # Arm trajectory data
        self.queued_arm_trajectory = None  # external arm trajectory (list of (t, joint_angles))
        self.arm_trajectory_start_time = None  # when arm trajectory playback started
        self.arm_trajectory_duration = 0.0  # total duration of current arm trajectory
        
        # Base trajectory data
        self.queued_base_trajectory = None  # external base trajectory (list of (t, base_pose))
        self.base_trajectory_start_time = None  # when base trajectory playback started
        self.base_trajectory_duration = 0.0  # total duration of current base trajectory
        
        # Current positions for smooth transitions
        self.current_arm_position = [0.0] * 6
        self.current_base_position = [0.0, 0.0, 0.0]
        
        # Mode tracking
        self.external_control_active = False

    def arm_trajectory_callback(self, msg):
        """Receive arm trajectory and store with proper timing information"""
        self.queued_arm_trajectory = []
        
        for point in msg.points:
            # Extract the actual time from the trajectory point
            time_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            self.queued_arm_trajectory.append((time_sec, point.positions))
        
        # Record when we start playing this trajectory
        self.arm_trajectory_start_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Get total trajectory duration
        if self.queued_arm_trajectory:
            self.arm_trajectory_duration = self.queued_arm_trajectory[-1][0]
        else:
            self.arm_trajectory_duration = 0.0
            
        self.external_control_active = True
        self.get_logger().info(f"Received ARM trajectory with {len(self.queued_arm_trajectory)} points, duration: {self.arm_trajectory_duration:.2f}s")

    def base_trajectory_callback(self, msg):
        """Receive base trajectory and store with proper timing information"""
        self.queued_base_trajectory = []
        
        for point in msg.points:
            # Extract the actual time from the trajectory point
            time_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            self.queued_base_trajectory.append((time_sec, point.positions))
        
        # Record when we start playing this trajectory
        self.base_trajectory_start_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Get total trajectory duration
        if self.queued_base_trajectory:
            self.base_trajectory_duration = self.queued_base_trajectory[-1][0]
        else:
            self.base_trajectory_duration = 0.0
            
        self.external_control_active = True
        self.get_logger().info(f"Received BASE trajectory with {len(self.queued_base_trajectory)} points, duration: {self.base_trajectory_duration:.2f}s")

    def interpolate_trajectory(self, trajectory, current_time):
        """Interpolate joint/base positions based on current time"""
        if not trajectory or len(trajectory) < 1:
            return None
            
        if len(trajectory) == 1:
            return trajectory[0][1]
        
        # Find the two trajectory points to interpolate between
        for i in range(len(trajectory) - 1):
            t1, pos1 = trajectory[i]
            t2, pos2 = trajectory[i + 1]
            
            if t1 <= current_time <= t2:
                # Linear interpolation between the two points
                if t2 - t1 > 0:
                    alpha = (current_time - t1) / (t2 - t1)
                else:
                    alpha = 0.0
                
                interpolated_pos = []
                for j in range(len(pos1)):
                    interp_val = pos1[j] * (1.0 - alpha) + pos2[j] * alpha
                    interpolated_pos.append(interp_val)
                
                return interpolated_pos
        
        # If we're past the end, return the last position
        if current_time >= trajectory[-1][0]:
            return trajectory[-1][1]
        
        # If we're before the start, return the first position
        return trajectory[0][1]

    def check_trajectory_completion(self):
        """Check if both trajectories are completed and update external control status"""
        t_now = self.get_clock().now().nanoseconds * 1e-9
        
        arm_completed = True
        base_completed = True
        
        if self.queued_arm_trajectory is not None and self.arm_trajectory_start_time is not None:
            arm_elapsed = t_now - self.arm_trajectory_start_time
            arm_completed = arm_elapsed >= self.arm_trajectory_duration + 0.5  # 0.5s buffer
            
        if self.queued_base_trajectory is not None and self.base_trajectory_start_time is not None:
            base_elapsed = t_now - self.base_trajectory_start_time
            base_completed = base_elapsed >= self.base_trajectory_duration + 0.5  # 0.5s buffer
        
        # If both trajectories are completed (or don't exist), disable external control
        if arm_completed and base_completed:
            if self.external_control_active:
                self.get_logger().info("All external trajectories completed. Switching to default mode.")
                self.external_control_active = False
                # Reset trajectory data
                self.queued_arm_trajectory = None
                self.queued_base_trajectory = None
                self.arm_trajectory_start_time = None
                self.base_trajectory_start_time = None

    def publish_trajectories(self):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        t = t_now - self.start_time

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

        # Check if trajectories are completed
        self.check_trajectory_completion()

        if self.external_control_active:
            # Handle arm trajectory
            if self.queued_arm_trajectory is not None and len(self.queued_arm_trajectory) > 0 and self.arm_trajectory_start_time is not None:
                arm_trajectory_time = t_now - self.arm_trajectory_start_time
                
                if arm_trajectory_time >= self.arm_trajectory_duration:
                    # Keep the last position - with safety check
                    joint_angles = self.queued_arm_trajectory[-1][1] if len(self.queued_arm_trajectory) > 0 else self.current_arm_position
                else:
                    # Interpolate current position based on trajectory timing
                    joint_angles = self.interpolate_trajectory(self.queued_arm_trajectory, arm_trajectory_time)
                    if joint_angles is None:
                        joint_angles = self.current_arm_position  # Fallback to current position
                
                self.current_arm_position = joint_angles
            else:
                # No valid arm trajectory, maintain current position
                joint_angles = self.current_arm_position
            
            # Handle Base trajectory
            if self.queued_base_trajectory is not None and len(self.queued_base_trajectory) > 0 and self.base_trajectory_start_time is not None:
                base_trajectory_time = t_now - self.base_trajectory_start_time
                
                if base_trajectory_time >= self.base_trajectory_duration:
                    # Keep the last position - with safety check
                    base_position = self.queued_base_trajectory[-1][1] if len(self.queued_base_trajectory) > 0 else self.current_base_position
                else:
                    # Interpolate current position based on trajectory timing
                    base_position = self.interpolate_trajectory(self.queued_base_trajectory, base_trajectory_time)
                    if base_position is None:
                        base_position = self.current_base_position  # Fallback to current position
                
                self.current_base_position = base_position
            else:
                # No valid base trajectory, maintain current position
                base_position = self.current_base_position
            
            # Create and publish arm trajectory point
            arm_point = JointTrajectoryPoint()
            arm_point.positions = joint_angles
            arm_point.time_from_start.sec = int(self.timer_period)
            arm_point.time_from_start.nanosec = int((self.timer_period % 1.0) * 1e9)
            arm_msg.points.append(arm_point)

            # Create and publish base trajectory point
            base_point = JointTrajectoryPoint()
            base_point.positions = base_position
            base_point.time_from_start.sec = int(self.timer_period)
            base_point.time_from_start.nanosec = int((self.timer_period % 1.0) * 1e9)
            base_msg.points.append(base_point)

            self.arm_pub.publish(arm_msg)
            self.base_pub.publish(base_msg)
            
            # Log progress occasionally
            if int(t * 2) % 10 == 0:  # Every 5 seconds
                arm_progress = 0.0
                base_progress = 0.0
                
                if self.queued_arm_trajectory is not None and len(self.queued_arm_trajectory) > 0 and self.arm_trajectory_duration > 0:
                    arm_elapsed = t_now - self.arm_trajectory_start_time if self.arm_trajectory_start_time else 0
                    arm_progress = min(100.0, (arm_elapsed / self.arm_trajectory_duration) * 100.0)
                    
                if self.queued_base_trajectory is not None and len(self.queued_base_trajectory) > 0 and self.base_trajectory_duration > 0:
                    base_elapsed = t_now - self.base_trajectory_start_time if self.base_trajectory_start_time else 0
                    base_progress = min(100.0, (base_elapsed / self.base_trajectory_duration) * 100.0)
                
                arm_status = f"ARM: {arm_progress:.1f}%" if self.queued_arm_trajectory else "ARM: N/A"
                base_status = f"BASE: {base_progress:.1f}%" if self.queued_base_trajectory else "BASE: N/A"
                self.get_logger().info(f'External control - {arm_status}, {base_status}')
            
            return  


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()