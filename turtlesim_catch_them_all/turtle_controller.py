#! /usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle, TurtleArray

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.turtle_to_catch = None
        self.pose = None
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "turtle1/cmd_vel",
            10
        )
        self.pose_subscriber = self.create_subscription(
            Pose,
            "turtle1/pose",
            self.callback_turtle_pose,
            10
        )  
        self.alive_turtles_subscriber = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def callback_turtle_pose(self, msg):
        self.pose = msg
    
    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            self.turtle_to_catch = msg.turtles[0]
            self.targetX = self.turtle_to_catch.x
            self.targetY = self.turtle_to_catch.y

    def control_loop(self):
        if self.pose is None or self.turtle_to_catch is None:
            return
        
        distX = self.targetX - self.pose.x
        distY = self.targetY - self.pose.y
        distance = math.sqrt(distX**2 + distY**2)

        msg = Twist()
        # 0.5 is the threshold to consider that the target has been reached
        if distance > 0.5:
            # The target has not been reached yet
            # position 
            msg.linear.x = distance * 2 
            # orientation
            desired_angle = math.atan2(distY, distX)
            diff = desired_angle - self.pose.theta

            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            msg.angular.z = 6 * diff
        else: 
            msg.linear.x = 0.0
            msg.angular.z = 0.0


        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()