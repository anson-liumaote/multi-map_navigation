import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_nav.sample_navigator import BasicNavigator
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
import threading
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from example_interfaces.srv import AddTwoInts
from rclpy.executors import MultiThreadedExecutor

class NavSrvServer(Node):
    def __init__(self):
        print('run node 1')
        super().__init__('id2id_srv_server')
        self.srv = self.create_service(AddTwoInts, 'id2id', self.id2id_callback)
        print('Nav service ready')

    def run(self):
        self.get_logger().info('Node1 is spinning')
        rclpy.spin(self)  # Spin this node

    def id2id_callback(self, request, response):
        self.mapServer(request.a, request.b)
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        print('callback')
        if FLAG_RUN:
            response.sum = 1
            return response
        
    def mapServer(self, id1, id2):
        global FLAG_RUN
        FLAG_RUN = True
        self.id_list = [1,2]

        
class NavHandler(Node):
    def __init__(self):
        super().__init__('nav_id2id_node')
        print('run node 2')
        # navigation init
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.goal_poses = []
        self.goal_pose = None
        self.goal_complete = False
        # load map init
        self.cli = self.create_client(LoadMap, 'map_server/load_map')
        self.init_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LoadMap.Request()
        self.declare_parameter('map_url_param', '')
        self.run()
        
        # service server init
        # self.result = False
        # self.id_list = []
        # self.srv = self.create_service(AddTwoInts, 'id2id', self.id2id_callback)
        # self.threadJob()
        

    # def id2id_callback(self, request, response):
    #     id_list = self.mapServer(request.a, request.b)
    #     self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
    #     print('callback')
    #     if self.result:
    #         response.sum = 1
    #         self.threadJob()
    #         return response


    def run(self):
        print('running runNavHandler')
        while True:
            if FLAG_RUN:
                self.setPose(161.94, 73.79, 0.0, 0.036)  # for navigation pose
                self.runNavigation()
                self.switchMap()

    def setPose(self, x, y, z=0.0, theta=1.0):
        print('set pose to:', x,y,z,theta)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = x
        self.goal_pose.pose.position.y = y
        self.goal_pose.pose.position.z = z
        self.goal_pose.pose.orientation.w = theta
    
    def runNavigation(self, time_out=1000000.0):
        cmd_send = self.navigator.goToPose(self.goal_pose)
        while not self.navigator.isNavComplete() and cmd_send:
            feed_back = self.navigator.getFeedback()
            print('Distance remaining: ' + '{:.2f}'.format(feed_back.distance_remaining) + ' meters.')
            ## Detect Time out ##
            if Duration.from_msg(feed_back.navigation_time) > Duration(seconds=time_out):
                print("Navigation TIme Out")
                self.navigator.cancelNav()
        nav_result = self.navigator.getResult()
        if nav_result == GoalStatus.STATUS_SUCCEEDED:
            print('Goal succeeded')
        elif nav_result == GoalStatus.STATUS_CANCELED:
            print('Goal was canceled')
        elif nav_result == GoalStatus.STATUS_ABORTED:
            print('Goal Failed')
        else:
            print('Goal has an invalid return status')

    def send_request(self):
        # self.req.map_url = self.get_parameter('map_url_param').get_parameter_value().string_value
        self.req.map_url = '/home/csl/Desktop/test_load_map_ws/src/load_map/maps/map.yaml'      # change map url
        print('loading map:', self.req.map_url)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def pubInitialPose(self, x, y ,z = 0.0, theta = 1.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = theta
        print("Initial pose set x: {}, y: {}, w: {}".format(x, y, theta))
        self.init_pose_publisher.publish(msg)
    
    def switchMap(self):
        response = self.send_request()
        if response.result==0:
            self.pubInitialPose(0.0, 0.0)     # for initial pose
            FLAG_RUN = False
            return True
        else:
            print("Set Initial Pose Failed")
        return False
    

        

def main(args=None):
    rclpy.init(args=args)
    nav_srv_server = NavSrvServer()
    nav_handler = NavHandler()
    
    # thread1 = threading.Thread(target=nav_srv_server.run)
    # thread1.start()

    # nav_srv_server.run()

    # nav_handler.threadJob()
    # print('thread job')

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(nav_srv_server)
    executor.add_node(nav_handler)

    executor.spin()
    # print('after spin')

    # rclpy.shutdown()
    # rclpy.spin(nav_srv_server)
    # print('spin nav_srv')
    # if nav_srv_server.result:
    #     nav_handler.run()
    
    # nav_handler.setPose(161.94, 73.79, 0.0, 0.036)  # for navigation pose
    # nav_handler.runNavigation()
    # nav_handler.switchMap()

    

    nav_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
