import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
 
class nodeName(Node):
    '''
    this node do this and this:
        subscribers: 
        publishers: 
    '''
    
    def __init__(self):
        super().__init__('node_name')
        self.get_logger().info("<node-name> initiated")
        self._init_members()
        self._init_subscribers()
        self._init_publishers()
        self._init_timers()
            
    def _init_members(self):
        self.msg = Odometry()
        self.declare_parameter("some_param", 1.0)
        self.some_param = self.get_parameter("some_param").value

    def _init_subscribers(self):
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._get, 10)
        self.get_logger().info("<node-name> subscribers initialized")
    
    def _init_publishers(self):
        self._publisher = self.create_publisher(Odometry, "/odom", 10)
        self.get_logger().info("<node-name> publishers initialized")

    def _init_timers(self):
        self._pub_timer = self.create_timer(1/10.0, self._publish_methoed)

    def _get(self,msg):
        pass
    
    def _publish_methoed(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = nodeName()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

