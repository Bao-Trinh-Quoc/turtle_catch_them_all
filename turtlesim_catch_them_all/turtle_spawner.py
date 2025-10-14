# !/usr/bin/env python3
from functools import partial

from requests import request
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
import math
from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.spawn_turtle_timer_ = self.create_timer(1.0, self.spawn_new_turtle)
        self.turtle_name_prefix =  "turtle"
        self.turtle_count = 1
        self.alive_turtles = []
        self.alive_turtles_publisher = self.create_publisher(TurtleArray, "alive_turtles", 10)
        self.get_logger().info("TurtleSpawnerNode has been started.")
        self.catch_turtle_service = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)

    def callback_catch_turtle(self, request, response):
        self.call_kill_server(request.name)
        response.success = True
        return response

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles 
        self.alive_turtles_publisher.publish(msg)

    def spawn_new_turtle(self):
        self.turtle_count += 1
        name = f"{self.turtle_name_prefix}{self.turtle_count}"
        x = random.uniform(0.0, 10.0)
        y = random.uniform(0.0, 10.0)
        theta = random.uniform(0.0, 2 * math.pi)
        self.call_spawn_server(x, y, theta, name)

    def call_spawn_server(self, x, y, theta, name):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for spawn service...")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_spawn, name=name, x=x, y=y, theta=theta)
        )

    def callback_spawn(self, future, name, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(f"Spawned turtle '{name}' at ({x}, {y}) with orientation {theta}")
                new_turtle = Turtle()
                new_turtle.name = name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.alive_turtles.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Failed to spawn turtle '{name}': {e}")

    def call_kill_server(self, name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for kill service...")
        
        request = Kill.Request() 
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_kill, name=name)
        )

    def callback_kill(self, future, name):
        try:
            response = future.result()
            for (i, turtle) in enumerate(self.alive_turtles):
                if turtle.name == name:
                    self.alive_turtles.pop(i)
                    self.publish_alive_turtles()
                    break

            self.get_logger().info(f"Killed turtle '{name}'")
        except Exception as e:
            self.get_logger().error(f"Failed to kill turtle '{name}': {e}")
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()