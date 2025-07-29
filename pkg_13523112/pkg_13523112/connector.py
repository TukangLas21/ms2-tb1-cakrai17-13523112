#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from queue import PriorityQueue
import threading

class Connector(Node):
    def __init__(self):
        super().__init__('connector')

        self.declare_parameter('autonomous_vel_topic', 'autonomous_vel')
        self.declare_parameter('joy_vel_topic', 'joy_vel')
        self.declare_parameter('keyboard_vel_topic', 'keyboard_vel')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('cmd_type_topic', 'cmd_type')
        self.declare_parameter('publish_rate', 1.0)
       
        # self.param_value = self.get_parameter('param_name').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        autonomous_topic = self.get_parameter('autonomous_vel_topic').get_parameter_value().string_value
        joy_topic = self.get_parameter('joy_vel_topic').get_parameter_value().string_value
        keyboard_topic = self.get_parameter('keyboard_vel_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        cmd_type_topic = self.get_parameter('cmd_type_topic').get_parameter_value().string_value
        
        self.priority_dict = {
            'joy': 1,
            'keyboard': 2,
            'autonomous': 3
        }
        
        self.publish_pq = PriorityQueue()
        self.thread_lock = threading.Lock()
        self.counter = 0
       
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
            self.subscriber_autonomous_callback,  # Callback function
            10                   # Queue size
        )
        
        self.joy_subscriber = self.create_subscription(
            Twist,
            joy_topic,
            self.subscriber_joy_callback,
            10
        )
        
        self.keyboard_subscriber = self.create_subscription(
            Twist,
            keyboard_topic,
            self.subscriber_keyboard_callback,  
            10
        )
       
        self.timer = self.create_timer(
            1.0 / publish_rate,  # Timer period (seconds)
            self.timer_callback  # Callback function
        )
       
        self.get_logger().info('Connector Node started!')
        
    def subscriber_autonomous_callback(self, msg):
        """Callback for subscriber"""
        with self.thread_lock:
            message = {
                'type': 'autonomous',
                'data': msg
            }
            self.publish_pq.put((self.priority_dict.get('autonomous', 0), self.counter, message))
            self.counter += 1
            
    def subscriber_joy_callback(self, msg):
        """Callback for joy subscriber"""
        with self.thread_lock:
            message = {
                'type': 'joy',
                'data': msg
            }
            self.publish_pq.put((self.priority_dict.get('joy', 0), self.counter, message))
            self.counter += 1
        
    def subscriber_keyboard_callback(self, msg):
        """Callback for keyboard subscriber"""
        with self.thread_lock:
            message = {
                'type': 'keyboard',
                'data': msg
            }
            self.publish_pq.put((self.priority_dict.get('keyboard', 0), self.counter, message))
            self.counter += 1

    def timer_callback(self):
        """Callback for timer"""
        with self.thread_lock:
            if not self.publish_pq.empty():
                _, _, message = self.publish_pq.get()
                vel_msg = message['data']
                type_msg = message['type']
                
                self.cmd_vel_publisher.publish(vel_msg)
                self.cmd_type_publisher.publish(String(data=type_msg))
                
                # self.get_logger().info(f'Published {type_msg} command: {vel_msg.linear.x}, {vel_msg.angular.z}')
        
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
