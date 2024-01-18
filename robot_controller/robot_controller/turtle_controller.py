#!/usr/bin/env pyhton3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial


class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.prev_x = 0
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)


    def pose_callback(self, pose: Pose):
        cmd = Twist()
        if pose.x > 5.5 and self.prev_x <= 5.5:
            self.prev_x = pose.x
            self.call_set_pen_services(255, 0, 0, 3, 0)
        elif pose.x <= 5.5 and self.prev_x >5.5:
            self.prev_x = pose.x
            self.call_set_pen_services(0, 0, 255, 3, 0)

        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publisher_.publish(cmd)


    def call_set_pen_services(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = SetPen.Request()
        request.r = r
        request.b = b
        request.g = g
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.set_pen_callback))

    def set_pen_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()