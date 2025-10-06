# !/usr/bin/env python3
from functools import partial
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
import math

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.spawn_turtle_timer_ = self.create_timer(2.0, self.spawn_new_turtle)
        self.turtle_name_prefix =  "turtle"
        self.turtle_count = 1
        self.get_logger().info("TurtleSpawnerNode has been started.")

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
        except Exception as e:
            self.get_logger().error(f"Failed to spawn turtle '{name}': {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()