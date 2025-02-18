import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32

class ServeurPortailTemperature(Node):
    def __init__(self):
        super().__init__('serveur_portail_temperature')
        self.service_bouton = self.create_service(Trigger, 'service_bouton', self.gerer_bouton)
        self.service_portail = self.create_service(Trigger, 'service_portail', self.gerer_portail)
        self.service_temperature = self.create_service(Trigger, 'service_temperature', self.gerer_temperature)
        self.publisher_temperature = self.create_publisher(Float32, '/donnees_temperature', 10)
        self.get_logger().info("Serveur prêt à recevoir des requêtes.")

    def gerer_bouton(self, requete, reponse):
        self.get_logger().info("Bouton appuyé, activation du portail.")
        reponse.success = True
        reponse.message = "Portail en cours d'activation."
        return reponse
    
    def gerer_portail(self, requete, reponse):
        self.get_logger().info("Commande de portail reçue.")
        reponse.success = True
        reponse.message = "Portail ouvert/fermé."
        return reponse
    
    def gerer_temperature(self, requete, reponse):
        temperature = float(input("Veuillez entrer une température en °C : "))
        self.publisher_temperature.publish(Float32(data=temperature))
        self.get_logger().info(f"Température simulée : {temperature:.2f}°C publiée sur /donnees_temperature")
        reponse.success = True
        reponse.message = "Température mise à jour."
        return reponse

def main(args=None):
    rclpy.init(args=args)
    noeud = ServeurPortailTemperature()
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        noeud.get_logger().info("Arrêt du serveur.")
    finally:
        noeud.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
