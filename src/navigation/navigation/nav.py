import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint
 
class Nav(Node):
    
    def __init__(self):
        super().__init__('nav')
        self.get_logger().info("nav initiated")
        self._init_publishers()
        self._init_subscribers()
            
    def _init_subscribers(self):
        self._coordinates_sub = self.create_subscription(NavSatFix, "/simple_drone/gps/nav", self.get_drone_coordinates, 10)
        self._target_sub = self.create_subscription(GeoPoint, "/target_location", self.get_target, 10)
        self.get_logger().info("nav subscribers initialized")
    
    def _init_publishers(self):
        self.coordinates_publisher = self.create_publisher(NavSatFix, "/robot_location", 10)
        self.get_logger().info("nav publishers initialized")

    def get_drone_coordinates(self, msg):
        drone_coordinates = msg
        self.coordinates_publisher.publish(drone_coordinates)
    
    def get_target(self, msg):
        geo = GeoPoint()
        
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

