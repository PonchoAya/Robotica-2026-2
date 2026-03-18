import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64

class NodeConverter(Node):
    def __init__(self):
        super().__init__('subscribe_node')

        self.counter_ = 0

        self.create_subscription(msg_type = Float64,
                                 topic = 'topic_1',
                                 qos_profile = 10,
                                 callback = self.sub_cbck)
        
        self.publisher_counter_ = self.create_publisher(msg_type = Float64,
                                                        topic = 'converter_topic',
                                                        qos_profile = 10)
        
        self.get_logger().info('Nodo subscritor activo')

    def sub_cbck(self, msg):
        self.counter_ = msg.data * 2 * math.pi
        new_msg = Float64()
        new_msg.data = self.counter_
        self.publisher_counter_.publish(new_msg)
        self.get_logger().info(f"Valor original: {msg.data}   Valor en rad/s: {new_msg.data}")


def main(args = None):
    rclpy.init(args = args)
    node = NodeConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()