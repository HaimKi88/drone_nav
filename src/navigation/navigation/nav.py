import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import time
from enum import Enum, auto
import numpy as np
from tf_transformations import euler_from_quaternion

class DroneState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    ROTATE = auto()
    MOVE_TO_POSITION = auto()
    LAND = auto()
    DONE = auto()

def geo_to_xy(lat, lon, lat0, lon0):
    # Earth radius in meters
    R = 6378137.0

    # Convert degrees to radians
    lat_rad = np.deg2rad(lat)
    lon_rad = np.deg2rad(lon)
    lat0_rad = np.deg2rad(lat0)
    lon0_rad = np.deg2rad(lon0)

    # Equirectangular projection
    dlat = lat_rad - lat0_rad
    dlon = lon_rad - lon0_rad

    x = R * dlon * np.cos(lat0_rad)
    y = R * dlat

    return x, y

class Nav(Node):
    
    def __init__(self):
        super().__init__('nav')
        self.get_logger().info("nav initiated")
        self._init_publishers()
        self._init_subscribers()
        
        self.state = DroneState.IDLE
        self.cmd_vel = Twist()
        self.linear_velocity = 1.5
        self.angular_velocity = 1.0
        self.yaw_p_ctrl = 0.5
        self.desired_yaw_err = np.deg2rad(1)
        self.desired_linear_err = 0.2
            
    def _init_subscribers(self):
        self._coordinates_sub = self.create_subscription(NavSatFix, "/simple_drone/gps/nav", self.get_drone_coordinates, 10)
        self._odom_sub = self.create_subscription(Odometry, "/simple_drone/odom", self.get_drone_odom, 10)
        self._target_sub = self.create_subscription(GeoPoint, "/target_location", self.get_target, 10)

        self.get_logger().info("nav subscribers initialized")
    
    def _init_publishers(self):
        self.coordinates_publisher = self.create_publisher(NavSatFix, "/robot_location", 10)
        self.takeoff_publisher = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, '/simple_drone/land', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.get_logger().info("nav publishers initialized")

    def get_drone_coordinates(self, msg):
        drone_coordinates = msg
        self.coordinates_publisher.publish(drone_coordinates)
    
    def get_drone_odom(self, msg):
        self.odom = msg
        
    def compute_yaw_error(self):
        dx = self.x_target - self.odom.pose.pose.position.x
        dy = self.y_target - self.odom.pose.pose.position.y
        yaw = np.arctan2(dy, dx)

        q_orientation = [self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, 
                         self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w]
        _, _, heading = euler_from_quaternion(q_orientation)
        self.get_logger().info(f'heading: {heading}, yaw: {yaw}')

        return heading-yaw
    
    def compute_linear_error(self):
        dx = self.x_target - self.odom.pose.pose.position.x
        dy = self.y_target - self.odom.pose.pose.position.y

        return np.hypot(dx,dy)

    def get_target(self, msg):
        self.get_logger().info(f'got a new target {msg}')
        self.target = msg
        lat0 = 32.072734
        lon0 = 34.787465
        x, y = geo_to_xy(self.target.latitude, self.target.longitude, lat0, lon0)
        self.x_target = x
        self.y_target = y

        self.state = DroneState.IDLE
        self._run_control_timer = self.create_timer(1/10.0, self.run_position_control)

    def run_position_control(self):
        self.get_logger().info(
            f"\nposition: [{self.odom.pose.pose.position.x:.2f}, {self.odom.pose.pose.position.y:.2f}, {self.odom.pose.pose.position.z:.2f}]\n"
            f"linear velocity: [{self.odom.twist.twist.linear.x:.2f}, {self.odom.twist.twist.linear.y:.2f}, {self.odom.twist.twist.linear.z:.2f}]\n"
            f"angular velocity: [{self.odom.twist.twist.angular.x:.2f}, {self.odom.twist.twist.angular.y:.2f}, {self.odom.twist.twist.angular.z:.2f}]")

        if self.state == DroneState.IDLE:
            self.get_logger().info("STATE IDLE")
            self.transition_to(DroneState.TAKEOFF)

        elif self.state == DroneState.TAKEOFF:
            self.takeoff()
            self.transition_to(DroneState.ROTATE)

        elif self.state == DroneState.ROTATE:
            if self.rotate():
                self.transition_to(DroneState.MOVE_TO_POSITION)

        elif self.state == DroneState.MOVE_TO_POSITION:
            if self.move_to_position():
                self.transition_to(DroneState.LAND)

        elif self.state == DroneState.LAND:
            self.land()
            self.transition_to(DroneState.DONE)
        
        elif self.state == DroneState.DONE:
            self._run_control_timer.destroy()

    def transition_to(self, new_state):
        self.get_logger().info(f"→ Transitioning to {new_state.name}")
        self.state = new_state

    def takeoff(self):
        self.takeoff_publisher.publish(Empty())
        self.get_logger().info("Taking off...")
        time.sleep(2) 

    def rotate(self):
        self.get_logger().info("Rotating in place...")
        yaw_err = self.compute_yaw_error()

        if abs(yaw_err) > self.desired_yaw_err:
            self.cmd_vel.angular.z = np.sign(yaw_err)*self.angular_velocity*yaw_err*self.yaw_p_ctrl
            self.cmd_vel_publisher.publish(self.cmd_vel)
            return False
        else:         
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.cmd_vel)
            self.get_logger().info(f'yaw error : {self.compute_yaw_error()}')
            return True

    def move_to_position(self):
        self.get_logger().info("Moving to target position...")
        
        linear_err = self.compute_linear_error()
        yaw_err = self.compute_yaw_error()

        if abs(linear_err) > self.desired_linear_err:
            
            self.cmd_vel.linear.x = np.sign(linear_err)*self.linear_velocity
            self.cmd_vel.angular.z = np.sign(yaw_err)*self.angular_velocity*yaw_err*0.05

            self.cmd_vel_publisher.publish(self.cmd_vel)
            self.get_logger().info(f'linear error : {self.compute_linear_error()}')
            return False
        else:         
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel_publisher.publish(self.cmd_vel)
            self.get_logger().info(f'linear error : {self.compute_linear_error()}')
            return True

    def land(self):
        self.get_logger().info("Landing...")
        time.sleep(2)  # Simulate landing
    
        
def main(args=None):
    rclpy.init(args=args)
    node = Nav()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

