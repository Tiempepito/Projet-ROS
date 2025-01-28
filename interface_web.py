import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from flask import Flask, request

# Création de l'application Flask
app = Flask(__name__)

# Classe pour le nœud ROS 2
class ServeurWebNode(Node):
    def __init__(self):
        super().__init__('serveur_web_node')
        self.pub_fenetre = self.create_publisher(String, '/commande_fenetre', 10)
        self.pub_led = self.create_publisher(Bool, '/commande_led', 10)
        self.get_logger().info("Serveur web ROS 2 prêt.")

    def publier_commande_fenetre(self, action):
        msg = String()
        msg.data = action
        self.pub_fenetre.publish(msg)
        self.get_logger().info(f"Commande fenêtre publiée : {action}")

    def publier_commande_led(self, etat):
        msg = Bool()
        msg.data = (etat == 'on')
        self.pub_led.publish(msg)
        self.get_logger().info(f"Commande LED publiée : {etat}")

# Routes Flask
@app.route('/controler_fenetre', methods=['POST'])
def controler_fenetre():
    node.publier_commande_fenetre(request.form['action'])
    return "Commande fenêtre envoyée"

@app.route('/controler_led', methods=['POST'])
def controler_led():
    node.publier_commande_led(request.form['etat'])
    return "Commande LED envoyée"

@app.route('/')
def index():
    return open('index.html').read()

# Fonction principale
def main(args=None):
    global node
    rclpy.init(args=args)
    node = ServeurWebNode()
    
    # Démarrer Flask dans un thread séparé
    import threading
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000)).start()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
