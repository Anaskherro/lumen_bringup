#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math

# --- CONFIGURATION ---
# Check if your board is on /dev/ttyACM0 or /dev/ttyUSB0
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

# Robot Physical Parameters
WHEEL_BASE = 0.245        # Distance between wheels in meters
TICKS_PER_METER = 981.5  # Encoder ticks per meter of travel

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # 1. Connect to Arduino
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
        
        # 2. Publishers (Odom & TF)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 3. Subscriber (Commands from Nav2)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        # 4. Odometry State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.prev_ticks_l = 0
        self.prev_ticks_r = 0
        
        self.first_run = True
        self.last_time = self.get_clock().now()

        # 5. Timer to read Serial (50Hz)
        self.create_timer(0.02, self.read_serial_data)

    def cmd_vel_cb(self, msg):
        """Sends velocity command to Arduino as 'linear_x,angular_z\n'"""
        if hasattr(self, 'ser') and self.ser.is_open:
            cmd = f"{msg.linear.x:.3f},{msg.angular.z:.3f}\n"
            try:
                self.ser.write(cmd.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")

    def read_serial_data(self):
        """Reads encoder ticks and updates odometry"""
        if not hasattr(self, 'ser') or not self.ser.is_open:
            return

        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                data = line.split(',')
                
                # Expecting ONLY 2 values: Left Ticks, Right Ticks
                if len(data) != 2: 
                    return 

                current_time = self.get_clock().now()
                
                # --- PARSE DATA ---
                try:
                    ticks_l = int(data[0])
                    ticks_r = int(data[1])
                except ValueError:
                    return # Skip bad data

                # Initialize on first run to avoid jumps
                if self.first_run:
                    self.prev_ticks_l = ticks_l
                    self.prev_ticks_r = ticks_r
                    self.last_time = current_time
                    self.first_run = False
                    return

                # Calculate time delta (dt)
                dt = (current_time - self.last_time).nanoseconds / 1e9
                if dt == 0: return

                # --- 16-BIT OVERFLOW LOGIC ---
                delta_ticks_l = ticks_l - self.prev_ticks_l
                delta_ticks_r = ticks_r - self.prev_ticks_r

                # Handle wrap-around (e.g. 32767 -> -32768)
                if delta_ticks_l < -30000:
                    delta_ticks_l += 65536
                elif delta_ticks_l > 30000:
                    delta_ticks_l -= 65536

                if delta_ticks_r < -30000:
                    delta_ticks_r += 65536
                elif delta_ticks_r > 30000:
                    delta_ticks_r -= 65536

                # Update previous ticks
                self.prev_ticks_l = ticks_l
                self.prev_ticks_r = ticks_r
                self.last_time = current_time

                # --- KINEMATICS ---
                # Convert ticks to distance
                d_left = delta_ticks_l / TICKS_PER_METER
                d_right = delta_ticks_r / TICKS_PER_METER

                # Calculate center distance and rotation
                d_dist = (d_left + d_right) / 2.0
                d_th = (d_right - d_left) / WHEEL_BASE

                # --- UPDATED INTEGRATION (Runge-Kutta 2nd Order) ---
                if d_dist != 0:
                    # Calculate x and y based on the MIDPOINT angle
                    # This is more accurate for curves than just using self.th
                    self.x += d_dist * math.cos(self.th + d_th / 2.0)
                    self.y += d_dist * math.sin(self.th + d_th / 2.0)
                
                # Update theta
                self.th += d_th

                # --- PUBLISH ODOMETRY ---
                odom = Odometry()
                odom.header.stamp = current_time.to_msg()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                
                # Position
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0.0
                
                # Orientation
                
                q_yaw = self.euler_to_quat(0, 0, self.th)
                odom.pose.pose.orientation = q_yaw
                
                # Velocity
                odom.twist.twist.linear.x = d_dist / dt
                odom.twist.twist.angular.z = d_th / dt
                odom.pose.covariance = [
                    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # X
                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # Y
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # Z
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # Roll
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # Pitch
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.03   # Yaw
                ]
                
                # Set velocity covariance (vx, vy, vz, vroll, vpitch, vyaw)
                odom.twist.covariance = [
                    0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # vx
                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # vy
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # vz
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # vroll
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # vpitch
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.03   # vyaw
                ]                
                self.odom_pub.publish(odom)

                # --- PUBLISH TF ---
                t = TransformStamped()
                t.header.stamp = current_time.to_msg()
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link"
                
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation = q_yaw
                self.tf_broadcaster.sendTransform(t)

            except Exception as e:
                self.get_logger().warn(f"Error in processing loop: {e}")
    
    def euler_to_quat(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
