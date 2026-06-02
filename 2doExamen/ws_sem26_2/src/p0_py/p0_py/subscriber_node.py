import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class NodeCounter(Node):
    def __init__(self):
        super().__init__('subscribe_node')

        self.counter_ = 0

        self.create_subscription(msg_type = Int64,
                                 topic = 'topic_1',
                                 qos_profile = 10,
                                 callback = self.sub_cbck)
        
        self.publisher_counter_ = self.create_publisher(msg_type = Int64,
                                                        topic = 'counter_topic',
                                                        qos_profile = 10)
        
        self.get_logger().info('Nodo subscritor activo')

    def sub_cbck(self, msg):
        self.counter_ += msg.data
        new_msg = Int64()
        new_msg.data = self.counter_
        self.publisher_counter_.publish(new_msg)
        self.get_logger().info(f"Suma acumulada: {self.counter_}")


def main(args = None):
    rclpy.init(args = args)
    node = NodeCounter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()