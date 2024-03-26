from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class NavSrvServer(Node):

    def __init__(self):
        super().__init__('id2id_srv_server')
        self.result = False
        self.srv = self.create_service(AddTwoInts, 'id2id', self.id2id_callback)

    def id2id_callback(self, request, response):
        id_list = self.mapServer(request.a, request.b)
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        print('callback')
        if self.result:
            response.sum = 1
            return response
        
    def mapServer(self, id1, id2):
        self.id_list = [1,2]
        self.result = True
        
def main():
    rclpy.init()

    minimal_service = NavSrvServer()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()