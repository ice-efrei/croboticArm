import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from ros_arm.Bras import Bras

class BrasNode(Node):
    def __init__(self):
        super().__init__('bras_node')
        self.bras = Bras()

        # Subscriber pour les coordonnées (x,y,z)
        self.sub_coord = self.create_subscription(
            Point,
            '/bras/coordonnees',
            self.listener_callback_coord,
            10
        )

        # Subscriber pour l'aimant (on/off)
        self.sub_aimant = self.create_subscription(
            Bool,
            '/bras/aimant',
            self.listener_callback_aimant,
            10
        )

        self.get_logger().info("✅ BrasNode initialisé et en attente de commandes...")

    def listener_callback_coord(self, msg: Point):
        self.get_logger().info(f"Reçu coord: x={msg.x}, y={msg.y}, z={msg.z}")
        self.bras.coordonnees(msg.x, msg.y, msg.z)

    def listener_callback_aimant(self, msg: Bool):
        state = 1 if msg.data else 0
        self.get_logger().info(f"Reçu commande aimant: {state}")
        self.bras.activerAimant(state)


def main(args=None):
    rclpy.init(args=args)
    node = BrasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
