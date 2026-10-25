#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from datetime import datetime

from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.time import Time


class OdometryTotalDistance(Node):
    def __init__(self):
        super().__init__('odometry_total_distance')
        self.publisher_ = self.create_publisher(Float32, '/total_distance', 10)
        
        self.lidar_odom_m = 0.0
        self.prev_lidar_msg = None
        
        self.wheel_odom_m = 0.0
        self.prev_wheel_msg = None
        
        self.tether_odom_m = 0.0
        self.prev_tether_msg = None
        
        self.sub_lidar_odom = self.create_subscription(
            Odometry,
            '/Odometry',
            self.lidar_odom_callback,
            10)
        
        self.sub_wheel_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.wheel_odom_callback,
            10)
        
        self.sub_tether_odom = self.create_subscription(
            Vector3,
            '/theter_odometry',
            self.tether_odom_callback,
            10)
        
        self.get_logger().info("odometry_total_distance started")
        
        LOOP_RATE_SECS = 2.0
        self.timer = self.create_timer(LOOP_RATE_SECS, self.publish_all)
        self.publisher_odom = self.create_publisher(Vector3, '/total_odom', 10)
        
    def publish_all(self):
        v3 = Vector3()
        
        v3.x = self.lidar_odom_m
        v3.y = self.wheel_odom_m
        v3.z = self.tether_odom_m
        
        self.publisher_odom.publish(v3)
        #self.get_logger().info(f"publishing: {v3}")
        
        if self.prev_lidar_msg:
            header_stamp_msg = self.prev_lidar_msg.header.stamp
            seconds = header_stamp_msg.sec
            nanoseconds = header_stamp_msg.nanosec
            time_in_seconds_double = seconds + nanoseconds * 1e-9

            now_datetime = datetime.fromtimestamp(time_in_seconds_double)
            formatted_time = now_datetime.strftime("%Y-%m-%d %H:%M:%S")
        
            self.get_logger().info(f'{formatted_time} {v3}')
        
    def lidar_odom_callback(self, msg):
        if not self.prev_lidar_msg:
            self.prev_lidar_msg = msg
            
        prev_x = self.prev_lidar_msg.pose.pose.position.x
        prev_y = self.prev_lidar_msg.pose.pose.position.y
            
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        d_increment = math.sqrt((curr_x - prev_x)**2  +(curr_y - prev_y)**2)
        self.lidar_odom_m += d_increment
            
        self.prev_lidar_msg  = msg
        
    def wheel_odom_callback(self, msg):
        if not self.prev_wheel_msg:
            self.prev_wheel_msg = msg
            
        prev_x = self.prev_wheel_msg.pose.pose.position.x
        prev_y = self.prev_wheel_msg.pose.pose.position.y
            
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        d_increment = math.sqrt((curr_x - prev_x)**2  +(curr_y - prev_y)**2)
        self.wheel_odom_m += d_increment
            
        self.prev_wheel_msg  = msg

    def tether_odom_callback(self, msg):
        if msg.x > 0:
            self.tether_odom_m = (msg.x / 1000)
    
def main(args=None):
    rclpy.init(args=args)

    odom_dist = OdometryTotalDistance()

    try:
        rclpy.spin(odom_dist)
    except (SystemExit, KeyboardInterrupt):
        rclpy.logging.get_logger("Quitting").info('Done') # pyright: ignore[reportAttributeAccessIssue]

    odom_dist.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()