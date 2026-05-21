#!urs/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import time
from math import cos,sin,atan2,acos,asin,sqrt,pow

class ScaraControl(Node):
    def __init__(self):
        super().__init__('scara_control_node')

        self.pub_joint01_ = self.create_publisher(msg_type=Float64,
            topic='/joint1/cmd_pos',qos_profile= 10)
        self.pub_joint02_ = self.create_publisher(msg_type=Float64,
            topic='/joint2/cmd_pos',qos_profile= 10)
        self.pub_joint03_ = self.create_publisher(msg_type=Float64,
            topic='/joint3/cmd_pos',qos_profile= 10)

        self.timer_control_=self.create_timer(timer_period_sec=1.0,
                        callback=self.cbck_scara_control)
        
        self.get_logger().info('Nodo controlador scara')
        
    def cbck_scara_control(self):
        # Definicion de las variables de la posicion inicial
        theta_0_1_p1 = Float64()
        theta_1_2_p1 = Float64()
        theta_2_3_p1 = Float64()
        # Definicion de las variables de la posicion final
        theta_0_1_p2 = Float64()
        theta_1_2_p2 = Float64()
        theta_2_3_p2 = Float64()

        #enviar comandos a la posicion inicial

        self.get_logger().info("Primera posición")

        theta_0_1_p1.data = float(1.1903)
        self.pub_joint01_.publish(theta_0_1_p1)

        theta_1_2_p1.data = float(2.0042)
        self.pub_joint02_.publish(theta_1_2_p1)

        theta_2_3_p1.data = float(-3.1945)
        self.pub_joint03_.publish(theta_2_3_p1)

        time.sleep(4.0)

        self.get_logger().info("Segunda posición")

        #enviar comandos a la posicion final
        theta_0_1_p2.data = float(-1.1903)
        self.pub_joint01_.publish(theta_0_1_p2)

        theta_1_2_p2.data = float(2.0042)
        self.pub_joint02_.publish(theta_1_2_p2)

        theta_2_3_p2.data = float(-0.8140)
        self.pub_joint03_.publish(theta_2_3_p2)

        time.sleep(1)

def cin_inv(x_in, y_in, theta_in, x_fin, y_fin, theta_fin):
    #Parámetros del robot
    L_1 = 0.5
    L_2 = 0.5
    L_3 = 0.3

    x_3_in = x_in - L_3 * cos(theta_in)
    y_3_in = y_in - L_3 * sin(theta_in)

    theta_2_in = acos(((x_3_in**2) + (y_3_in**2) - (L_1**2) - (L_2**2)) / (2*L_1*L_2))
    beta = atan2(y_3_in,x_3_in)
    alpha = acos((x_3_in**2 + y_3_in**2 + L_1**2 - L_2**2) / (2*L_1*sqrt(x_3_in**2 + )))


    return theta_2_in

def main(args= None):
    rclpy.init(args=args)
    node = ScaraControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== '_main_':
    main()