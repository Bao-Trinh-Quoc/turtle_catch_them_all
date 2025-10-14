#! /usr/bin/env python3

import turtle
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from functools import partial
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_closest_turtle", True)

        self.turtle_to_catch = None
        self.catch_closest_turtle = self.get_parameter("catch_closest_turtle").value
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
            if self.catch_closest_turtle:
                closest_turtle = None
                closest_distance = None

                for turtle in msg.turtles:
                    distX = turtle.x - self.pose.x
                    distY = turtle.y - self.pose.y
                    distance = math.sqrt(distX**2 + distY**2)

                    if closest_distance is None or distance < closest_distance:
                        closest_distance = distance
                        closest_turtle = turtle
                
                self.turtle_to_catch = closest_turtle
            else:
                self.turtle_to_catch = msg.turtles[0]

        if self.turtle_to_catch is not None:
            self.targetX = self.turtle_to_catch.x
            self.targetY = self.turtle_to_catch.y

    def call_catch_turtle_server(self, name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for catch_turtle service...")
        
        request = CatchTurtle.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_catch_turtle, name=name)
        )

    def callback_catch_turtle(self, future, name): 
        try:
            response = future.result()
            if not response.success:
                self.get_logger().info(f"Failed to catch turtle '{name}'")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

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
            # Target reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch.name)
            self.turtle_to_catch = None


        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()