#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Connector(Node):
    def __init__(self):
        super().__init__('connector')

        self.declare_parameter('autonomous_vel_topic', 'autonomous_vel')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('cmd_type_topic', 'cmd_type')
        self.declare_parameter('publish_rate', 1.0)
       
        # self.param_value = self.get_parameter('param_name').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        autonomous_topic = self.get_parameter('autonomous_vel_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        cmd_type_topic = self.get_parameter('cmd_type_topic').get_parameter_value().string_value
       
        self.autonomous_val = None
       
        self.cmd_vel_publisher = self.create_publisher(
            Twist,           # Message type
            cmd_vel_topic,    # Topic name
            10               # Queue size
        )
        
        self.cmd_type_publisher = self.create_publisher(
            String,
            cmd_type_topic,
            10
        )
       
        self.autonomous_subscriber = self.create_subscription(
            Twist,              # Message type
            autonomous_topic,       # Topic name
            self.subscriber_callback,  # Callback function
            10                   # Queue size
        )
       
        self.timer = self.create_timer(
            1.0 / publish_rate,  # Timer period (seconds)
            self.timer_callback  # Callback function
        )
       
        self.get_logger().info('Connector Node started!')
        
    def subscriber_callback(self, msg):
        """Callback for subscriber"""
        self.autonomous_val = msg

    def timer_callback(self):
        """Callback for timer"""
        self.cmd_type_publisher.publish(String(data='autonomous'))
        if self.autonomous_val:
            self.cmd_vel_publisher.publish(self.autonomous_val)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = Connector()
   
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
