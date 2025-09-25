#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
import sys

class TestLoopNode(Node):
    def __init__(self, axis: str, increment: float):
        super().__init__('test_loop_node')
        self.pub = self.create_publisher(Point, '/bras/coordonnees', 10)
        self.coord = Point()
        self.coord.x = 0.0
        self.coord.y = 20.0
        self.coord.z = 20.0
        self.axis = axis.lower()
        self.increment = float(increment)
        self.get_logger().info(f"TestLoopNode lancé : incrémente {self.axis} de {self.increment} en boucle")

    def run_loop(self, delay=0.5):
        try:
            while rclpy.ok():
                if self.axis == 'x':
                    self.coord.x = float(self.coord.x + self.increment)
                elif self.axis == 'y':
                    self.coord.y = float(self.coord.y + self.increment)
                elif self.axis == 'z':
                    self.coord.z = float(self.coord.z + self.increment)
                else:
                    self.get_logger().error(f"Axe inconnu: {self.axis}")
                    break

                self.pub.publish(self.coord)
                self.get_logger().info(f"Envoi coord: x={self.coord.x:.1f}, y={self.coord.y:.1f}, z={self.coord.z:.1f}")
                time.sleep(delay)
        except KeyboardInterrupt:
            self.get_logger().info("Arrêt du loop")

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print("Usage: ros2 run ros_arm test_loop <axe> <increment>")
        print("Exemple: ros2 run ros_arm test_loop x +1")
        return

    axis = sys.argv[1]
    try:
        increment = float(sys.argv[2])
    except ValueError:
        print("Increment doit être un nombre, par exemple +1 ou -1")
        return

    node = TestLoopNode(axis, increment)
    try:
        node.run_loop(delay=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
