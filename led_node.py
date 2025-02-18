import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

class ServeurROS(Node):
    def __init__(self):
        super().__init__('serveur_ros')
        
        # Service pour gérer la LED
        self.srv = self.create_service(Trigger, 'toggle_led', self.led_callback)
        
        # Publisher pour la LED
        self.publisher_led = self.create_publisher(Bool, '/commande_led', 10)
        
        self.etat_led = False  # LED initialement éteinte
        self.get_logger().info("Serveur ROS prêt à contrôler la LED.")
    
    def led_callback(self, request, response):
        self.etat_led = not self.etat_led  # Toggle état de la LED
        
        if self.etat_led:
            self.get_logger().info("LED Allumée.")
            self.publisher_led.publish(Bool(data=True))
            response.message = "LED allumée."
        else:
            self.get_logger().info("LED Éteinte.")
            self.publisher_led.publish(Bool(data=False))
            response.message = "LED éteinte."
        
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServeurROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du serveur.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
