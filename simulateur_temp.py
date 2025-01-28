import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SimulateurTemperature(Node):

    def __init__(self):
        super().__init__('simulateur_temperature')
        self.publicateur = self.create_publisher(Float32, '/donnees_temperature', 10)
        self.get_logger().info("Simulateur de capteur de température démarré.")
        self.timer = self.create_timer(1.0, self.publication_temperature)

    def publication_temperature(self):
        try:
            temperature = float(input("Veuillez entrer une température en °C : "))
            self.publicateur.publish(Float32(data=temperature))
            self.get_logger().info(f"Température simulée : {temperature:.2f}°C publiée sur /donnees_temperature")
        except ValueError:
            self.get_logger().error("Erreur : Veuillez entrer une valeur numérique valide.")

def main(args=None):
    rclpy.init(args=args)
    simulateur = SimulateurTemperature()
    try:
        rclpy.spin(simulateur)
    except KeyboardInterrupt:
        simulateur.get_logger().info("Arrêt du simulateur de température.")
    finally:
        simulateur.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

