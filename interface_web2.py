import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger  # Service ROS2 pour la LED
from flask import Flask, request, jsonify
import threading

# Création de l'application Flask
app = Flask(__name__)

# Classe pour le nœud ROS2
class ServeurWebNode(Node):
    def __init__(self):
        super().__init__('serveur_web_node')

        # Publisher pour les fenêtres
        self.pub_fenetre = self.create_publisher(String, '/commande_fenetre', 10)

        # Client pour le service de la LED
        self.client_led = self.create_client(Trigger, 'toggle_led')

        # Subscription pour la température
        self.subscription_temp = self.create_subscription(
            Float32, '/donnees_temperature', self.recevoir_temperature, 10)

        self.temperature_actuelle = "--"

        self.get_logger().info("Serveur web ROS2 prêt.")

    def publier_commande_fenetre(self, action):
        msg = String()
        msg.data = action
        self.pub_fenetre.publish(msg)
        self.get_logger().info(f"Commande fenêtre publiée : {action}")

    def activer_led(self):
        """ Appelle le service ROS2 pour allumer/éteindre la LED sans bloquer """
        req = Trigger.Request()
        future = self.client_led.call_async(req)
        future.add_done_callback(self.reponse_led)

    def reponse_led(self, future):
        """ Callback pour gérer la réponse du service LED """
        try:
            response = future.result()
            self.get_logger().info(f"Réponse du service LED : {response.message}")
        except Exception as e:
            self.get_logger().error(f"Erreur service LED : {str(e)}")

    def recevoir_temperature(self, msg):
        self.temperature_actuelle = f"{msg.data:.2f}"
        self.get_logger().info(f"Température reçue : {self.temperature_actuelle}°C")

# Initialisation du nœud ROS
node = None

# Routes Flask
@app.route('/temperature')
def get_temperature():
    global node
    if node:
        return jsonify({'temperature': node.temperature_actuelle})
    else:
        return jsonify({'error': "Nœud ROS non disponible"}), 500

@app.route('/controler_fenetre', methods=['POST'])
def controler_fenetre():
    global node
    if node:
        action = request.form['action']
        node.publier_commande_fenetre(action)
        return jsonify({"message": f"Commande {action} envoyée avec succès"})
    else:
        return jsonify({"error": "Nœud ROS non disponible"}), 500

@app.route('/controler_led', methods=['POST'])
def controler_led():
    global node
    if node:
        node.activer_led()
        return jsonify({"message": "Commande LED envoyée"})
    else:
        return jsonify({"error": "Nœud ROS non disponible"}), 500

@app.route('/')
def index():
    return open('index.html').read()

# Fonction pour démarrer Flask dans un thread séparé
def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)

# Fonction principale
def main(args=None):
    global node
    rclpy.init(args=args)
    node = ServeurWebNode()
    
    # Démarrer Flask dans un thread séparé
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

