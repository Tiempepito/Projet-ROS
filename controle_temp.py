import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ControleTemperatureNode(Node):
    def __init__(self):
        super().__init__('controle_temperature_node')
        self.subscription = self.create_subscription(Float32,'/donnees_temperature',self.analyser_temperature,10)
        self.get_logger().info("En attente des données...")

    def analyser_temperature(self, msg):
        temperature = msg.data
        if temperature > 25.0:
            self.get_logger().info(f"Température élevée détectée ({temperature:.2f}°C). Ouvrir les fenêtres.")
        elif temperature < 15.0:
            self.get_logger().info(f"Température très basse détectée ({temperature:.2f}°C). Allumer le chauffage, LED ROUGE.")
        elif temperature < 20.0:
            self.get_logger().info(f"Température basse détectée ({temperature:.2f}°C). Fermer les fenêtres.")
        elif temperature >= 20.0:
            self.get_logger().info(f"Température Bonne détectée ({temperature:.2f}°C).Température ambiante Bonne , LED VERTE..")
        if temperature >= 50.0:
            self.get_logger().info(f"Température très élevée ({temperature:.2f}°C). Urgence à traiter.")

def main(args=None):
    rclpy.init(args=args)
    node = ControleTemperatureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du nœud de contrôle de température.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

