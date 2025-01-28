import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class LedNode(Node):
    def __init__(self):
        super().__init__('led_node')
        self.subscription_led = self.create_subscription(Bool, '/commande_led', self.controler_led, 10)
        self.subscription_bouton = self.create_subscription(Bool, '/etat_bouton', self.reagir_bouton, 10)
        self.get_logger().info("Nœud LED prêt.")

    def controler_led(self, msg):
        if msg.data:
            self.get_logger().info("LED allumée.")
        else:
            self.get_logger().info("LED éteinte.")

    def reagir_bouton(self, msg):
        if msg.data:
            self.get_logger().info("Bouton appuyé, allumage de la LED.")
            self.controler_led(Bool(data=True))
        else:
            self.get_logger().info("Bouton relâché, extinction de la LED.")
            self.controler_led(Bool(data=False))

def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du nœud LED.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
