import rclpy
from rclpy.node import Node
import threading

class Node1(Node):
    def __init__(self):
        super().__init__('node1')
        # Add your node initialization here

    def run(self):
        self.get_logger().info('Node1 is spinning')
        rclpy.spin(self)  # Spin this node

class Node2(Node):
    def __init__(self):
        super().__init__('node2')
        # Add your node initialization here

    def run(self):
        self.get_logger().info('Node2 is running')

def main(args=None):
    rclpy.init(args=args)

    node1 = Node1()
    node2 = Node2()

    # Define a thread for node1's spinning loop
    thread1 = threading.Thread(target=node1.run)

    # Start the thread for node1's spinning loop
    thread1.start()

    # Execute node2's functionality in the main thread
    node2.run()

    # Wait for node1's spinning loop to finish
    thread1.join()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
