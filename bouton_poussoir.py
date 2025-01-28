import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BoutonPoussoirNode(Node):
    def __init__(self):
        super().__init__('bouton_poussoir_node')
        self.publisher = self.create_publisher(Bool, '/etat_bouton', 10)
        self.timer = self.create_timer(1.0, self.publicher_etat_bouton)

    def publicher_etat_bouton(self):
        etat = bool(int(input("Entrez l'état du bouton poussoir (0 pour relâché, 1 pour appuyé) : ")))
        self.publisher.publish(Bool(data=etat))
        self.get_logger().info(f"État du bouton poussoir publié : {etat}")

def main(args=None):
    rclpy.init(args=args)
    node = BoutonPoussoirNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du nœud bouton poussoir.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
