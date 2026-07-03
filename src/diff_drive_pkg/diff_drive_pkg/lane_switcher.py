import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu 
import time 

# Helper function to convert Quaternion to Yaw (rotation around Z-axis)
def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z 

class LaneSwitcher(Node):
    
    def __init__(self):
        super().__init__('lane_switcher')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscriptions
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.listener_callback, qos_policy)
            
        self.imu_subscription = self.create_subscription(
            Imu, '/imu', self.imu_callback, qos_policy)

        # --- State Variables ---
        self.min_distance = float('inf')
        self.current_yaw = 0.0 
        self.current_lane = 'right'
        self.switching = False
        self.switch_start_time = 0
        self.cooldown_time = 0 
        self.target_yaw = 0.0 
        
        # --- CONTROL PARAMETERS (FINAL TUNING) ---
        self.linear_speed = 0.5           
        self.angular_speed_turn = 1.0     # Max angular force
        self.obstacle_distance = 1.5      
        self.P_GAIN = 0.4                 # P-gain for IMU stabilization
        
        # MANEUVER TIMING (CRITICALLY TUNED)
        self.TURN_TO_ANGLE_DURATION = 0.8 # FIX 1: Time to pivot 45 degrees
        self.CROSS_DURATION = 1.8         # FIX 2: Time to drive across the lane
        self.COOLDOWN_DELAY = 4.0
        self.ANGLE_SETPOINT = math.pi / 4.0 # FIX 1: Target 45 degrees
        
        self.timer = self.create_timer(0.05, self.control_loop_callback)
        self.get_logger().info('Lane Switcher Started with Final Arc Tuning! ')

    def imu_callback(self, msg):
        """Processes IMU data to get current yaw angle."""
        self.current_yaw = euler_from_quaternion(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )

    def listener_callback(self, msg):
        """Processes LiDAR data, filtering for the front sector."""
        center_idx = len(msg.ranges) // 2
        window = 10 
        front_ranges = msg.ranges[center_idx - window : center_idx + window]

        valid_ranges = [r for r in front_ranges if not math.isinf(r) and r > 0.1 and r < 12.0 and r != 0.0]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
        else:
            self.min_distance = float('inf')

    def control_loop_callback(self):
        cmd = Twist()
        now = self.get_clock().now().nanoseconds
        cooldown_elapsed = (now - self.cooldown_time) / 1e9 

        if self.switching:
            self.switch_lane_action()
            return

        # 1. Yaw Stabilization (P-Controller to 0.0 rad)
        error = 0.0 - self.current_yaw
        angular_correction = self.P_GAIN * error

        # 2. Trigger switch if obstacle is close AND cooldown is over
        if self.min_distance < self.obstacle_distance and cooldown_elapsed > self.COOLDOWN_DELAY: 
            
            self.get_logger().warn(f"OBSTACLE DETECTED at {self.min_distance:.2f}m! Initiating Arc...")
            
            # Initiate state change
            self.switching = True
            self.switch_start_time = now
            self.target_lane = 'left' if self.current_lane == 'right' else 'right'
            
            # Calculate the target angle for the crossing phase
            turn_direction = self.ANGLE_SETPOINT if self.target_lane == 'left' else -self.ANGLE_SETPOINT
            self.target_yaw = self.current_yaw + turn_direction 
            
            self.cooldown_time = now # Reset cooldown timer
            
            self.switch_lane_action() 
            
        else:
            # Drive straight + Apply stabilization
            cmd.linear.x = self.linear_speed
            cmd.angular.z = angular_correction
            self.publisher_.publish(cmd)

    def switch_lane_action(self):
        elapsed_time = (self.get_clock().now().nanoseconds - self.switch_start_time) / 1e9
        cmd = Twist()
        
        # --- STAGE 1: TURN TO TARGET ANGLE (0.0s to 0.8s) ---
        if elapsed_time < self.TURN_TO_ANGLE_DURATION:
            # Turn aggressively to the stored target angle (self.target_yaw)
            error = self.target_yaw - self.current_yaw
            
            cmd.linear.x = 0.1 # Slow forward speed during turn
            cmd.angular.z = self.P_GAIN * 1.5 * error # Aggressive P-Control to hit 45 degrees
            
            self.publisher_.publish(cmd)
            
        # --- STAGE 2: CROSS THE LANE (0.8s to 2.6s) ---
        elif elapsed_time < (self.TURN_TO_ANGLE_DURATION + self.CROSS_DURATION):
            
            # Drive straight along the current heading to cross the lane.
            
            # Apply light stabilization to maintain the 45-degree crossing angle
            error = self.target_yaw - self.current_yaw
            
            cmd.linear.x = self.linear_speed + 0.3 # FIX 2: Increased speed (0.8 m/s) during cross
            cmd.angular.z = self.P_GAIN * 0.5 * error # Gentle P-Control to maintain the angle
            
            self.publisher_.publish(cmd)
            
        # --- STAGE 3: MANEUVER COMPLETE (Transition to Stabilization) ---
        else:
            # Reset state, and the main control loop will immediately start stabilizing to 0.0
            self.switching = False
            self.current_lane = self.target_lane
            self.get_logger().info(f"Switch Complete. New Lane: {self.current_lane}")
# --- ROS 2 Entry Point ---
def main(args=None):
    rclpy.init(args=args)
    node = LaneSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()