#!usr/bin/env python3

import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Float64

class MyNode(Node):
    def __init__(self):
        super().__init__("my_node")
        self.number_ = random.random()
        self.publicador_ = self.create_publisher(msg_type = Float64, 
                                                  topic = 'topic_1', 
                                                  qos_profile = 10)
        
        self.timer = self.create_timer(timer_period_sec = 1.0, 
                                       callback = self.cbk)
        
        self.get_logger().info('Nodo publicador activo')
    
    def cbk(self):
        msg = Float64()
        msg.data = random.random()
        self.publicador_.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    nodo1 = MyNode()
    rclpy.spin(nodo1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()