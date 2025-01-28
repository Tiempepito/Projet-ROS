import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class FenetreNode(Node):
    def __init__(self):
        super().__init__('fenetre_node')
        
        # Création du cerveau moteur
        self.moteur_fenetre = MoteurFenetre()

        # Subscribers
        self.subscription_fenetre = self.create_subscription(String, '/commande_fenetre', self.controler_fenetre, 10)
        self.subscription_temperature = self.create_subscription(Float32, '/donnees_temperature', self.reagir_temperature, 10)
        
        self.get_logger().info("Nœud fenêtre prêt avec cerveau moteur.")

    def controler_fenetre(self, msg):
        if msg.data == "ouvrir":
            self.moteur_fenetre.ouvrir()
            self.get_logger().info("Commande reçue : Ouverture des fenêtres.")
        elif msg.data == "fermer":
            self.moteur_fenetre.fermer()
            self.get_logger().info("Commande reçue : Fermeture des fenêtres.")
        else:
            self.get_logger().warn(f"Commande inconnue reçue : {msg.data}")

    def reagir_temperature(self, msg):
        if msg.data > 25.0:  # Si la température dépasse 25°C, ouvrir les fenêtres
            self.get_logger().info(f"Température élevée ({msg.data}°C), ouverture des fenêtres.")
            self.moteur_fenetre.ouvrir()
        elif msg.data < 20.0:  # Si la température descend en dessous de 20°C, fermer les fenêtres
            self.get_logger().info(f"Température basse ({msg.data}°C), fermeture des fenêtres.")
            self.moteur_fenetre.fermer()

class MoteurFenetre:
    def __init__(self):
        self.etat_fenetre = "fermée"  # État initial des fenêtres

    def ouvrir(self):
        if self.etat_fenetre != "ouverte":
            self.etat_fenetre = "ouverte"
            print("Action : Fenêtres ouvertes.")
        else:
            print("Les fenêtres sont déjà ouvertes.")

    def fermer(self):
        if self.etat_fenetre != "fermée":
            self.etat_fenetre = "fermée"
            print("Action : Fenêtres fermées.")
        else:
            print("Les fenêtres sont déjà fermées.")

    def get_etat(self):
        return self.etat_fenetre

def main(args=None):
    rclpy.init(args=args)
    node = FenetreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du nœud fenêtre.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
