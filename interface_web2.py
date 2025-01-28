import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from flask import Flask, request, redirect, url_for
import threading

# Création de l'application Flask
app = Flask(__name__)

# Classe pour le nœud ROS 2
class ServeurWebNode(Node):
    def __init__(self):
        super().__init__('serveur_web_node')
        
        # Publishers pour les topics ROS 2
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
    action = request.form['action']
    node.publier_commande_fenetre(action)
    return redirect(url_for('index'))

@app.route('/controler_led', methods=['POST'])
def controler_led():
    etat = request.form['etat']
    node.publier_commande_led(etat)
    return redirect(url_for('index'))

@app.route('/')
def index():
    return open('index.html').read()

# Fonction pour démarrer Flask dans un thread séparé
def run_flask():
    app.run(host='0.0.0.0', port=5000)

# Fonction principale
def main(args=None):
    global node
    rclpy.init(args=args)
    node = ServeurWebNode()
    
    # Démarrer Flask dans un thread séparé
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True  # Le thread se fermera lorsque le programme principal se termine
    flask_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du serveur web ROS 2.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
