import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import requests

class MiNodo(Node):
    def __init__(self):
        super().__init__('mi_nodo')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('He recibido: [%s, %s]' % (msg.linear.x, msg.angular.z))
        level = 0.1
        dir = 'http://mi-url.com/'
        if msg.angular.z > 0.1:
            self.get_logger().info('El robot está girando a la izquierda')
            response = requests.get(dir+'left')
        elif msg.angular.z < -0.1:
            self.get_logger().info('El robot está girando a la derecha')
            response = requests.get(dir+'right')
        else:
            response = requests.get(dir+'normal')
            self.get_logger().info('El robot está avanzando recto')

def main(args=None):
    rclpy.init(args=args)
    nodo = MiNodo()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass

    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
