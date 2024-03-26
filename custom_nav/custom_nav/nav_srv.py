import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from load_map.sample_navigator import BasicNavigator
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
import threading
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionServer

from example_interfaces.srv import AddTwoInts
import os


import time

# ros2 action send_goal nav_action example_interfaces/action/Fibonacci "{order: 1}"

class NavHandler(Node):
    def __init__(self):
        super().__init__('nav_srv_node')
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
        self.declare_parameter('map_numbers', 0)
        self.map_numbers = self.get_parameter('map_numbers').get_parameter_value().integer_value
        for i in range(self.map_numbers):
            self.declare_parameter('map_{}.map_url_param'.format(str(i)), '')
            self.declare_parameter('map_{}.initial_pose.x'.format(str(i)), 0.0)
            self.declare_parameter('map_{}.initial_pose.y'.format(str(i)), 0.0)
            self.declare_parameter('map_{}.initial_pose.z'.format(str(i)), 0.0)
            self.declare_parameter('map_{}.initial_pose.yaw'.format(str(i)), 1.0)
            self.declare_parameter('map_{}.nav_pose.x'.format(str(i)), 0.0)
            self.declare_parameter('map_{}.nav_pose.y'.format(str(i)), 0.0)
            self.declare_parameter('map_{}.nav_pose.z'.format(str(i)), 0.0)
            self.declare_parameter('map_{}.nav_pose.yaw'.format(str(i)), 1.0)
        
        self.srv = self.create_service(AddTwoInts, 'id2id', self.id2id_callback)
        self.get_logger().info('service server ready')
        # while True:
        #     cmd = int(input('CMD:'))
        #     if cmd==1:
        #         self.setPose(155.94, 78.79, 0.0, 0.036)  # for navigation pose
        #         self.runNavigation()
        #     elif cmd==2:
        #         self.switchMap()

    def id2id_callback(self, request, response):
        self.mapServer(request.a, request.b)
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        if request.a == 1:
            self.req.map_url = self.get_parameter('map_{}.map_url_param'.format(str(request.b))).get_parameter_value().string_value
            self.initial_pose_x = self.get_parameter('map_{}.initial_pose.x'.format(str(request.b))).get_parameter_value().double_value
            self.initial_pose_y = self.get_parameter('map_{}.initial_pose.y'.format(str(request.b))).get_parameter_value().double_value
            self.initial_pose_z = self.get_parameter('map_{}.initial_pose.z'.format(str(request.b))).get_parameter_value().double_value
            self.initial_pose_yaw = self.get_parameter('map_{}.initial_pose.yaw'.format(str(request.b))).get_parameter_value().double_value
            # self.switchMap()
            self.send_request()
            self.pubInitialPose()
        elif request.a == 2:
            self.nav_pose_x = self.get_parameter('map_{}.nav_pose.x'.format(str(request.b))).get_parameter_value().double_value
            self.nav_pose_y = self.get_parameter('map_{}.nav_pose.y'.format(str(request.b))).get_parameter_value().double_value
            self.nav_pose_z = self.get_parameter('map_{}.nav_pose.z'.format(str(request.b))).get_parameter_value().double_value
            self.nav_pose_yaw = self.get_parameter('map_{}.nav_pose.yaw'.format(str(request.b))).get_parameter_value().double_value
            self.setPose()  # for navigation pose
            self.runNavigation()
        
        response.sum = 1
        return response
        

    def mapServer(self, id1, id2):
        self.get_logger().info('Running map server')
        self.id_list = [1,2]

    def setPose(self):
        # print('set pose to:', self.nav_pose_x,self.nav_pose_y,self.nav_pose_z,self.nav_pose_yaw)
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = self.nav_pose_x
        self.goal_pose.pose.position.y = self.nav_pose_y
        self.goal_pose.pose.position.z = self.nav_pose_z
        self.goal_pose.pose.orientation.w = self.nav_pose_yaw
    
    def runNavigation(self, time_out=1000000.0):
        self.navigator.goToPose(self.goal_pose)
        # cmd_send = self.navigator.goToPose(self.goal_pose)
        # while not self.navigator.isNavComplete() and cmd_send:
        #     feed_back = self.navigator.getFeedback()
        #     print('Distance remaining: ' + '{:.2f}'.format(feed_back.distance_remaining) + ' meters.')
        #     ## Detect Time out ##
        #     if Duration.from_msg(feed_back.navigation_time) > Duration(seconds=time_out):
        #         print("Navigation TIme Out")
        #         self.navigator.cancelNav() this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
        #     print('Goal succeeded')
        # elif nav_result == GoalStatus.STATUS_CANCELED:
        #     print('Goal was canceled')
        # elif nav_result == GoalStatus.STATUS_ABORTED:
        #     print('Goal Failed')
        # else:
        #     print('Goal has an invalid return status')

    def send_request(self):
        # self.req.map_url = self.get_parameter('map_1.map_url_param').get_parameter_value().string_array_value
        self.get_logger().info('loading map:'+ str(self.req.map_url))
        # self.future = self.cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()
        os.system('ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "map_url: {}"'.format(str(self.req.map_url)))
        self.get_logger().info('complete')
        
    
    def pubInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.initial_pose_x
        msg.pose.pose.position.y = self.initial_pose_y
        msg.pose.pose.position.z = self.initial_pose_z
        msg.pose.pose.orientation.w = self.initial_pose_yaw
        self.get_logger().info("Initial pose set x: {}, y: {}, w: {}".format(self.initial_pose_x, self.initial_pose_y, self.initial_pose_yaw))
        self.init_pose_publisher.publish(msg)
    
    # def switchMap(self):
        # self.send_request()
        # self.initial_pose = self.get_parameter('map_1.initial_pose').get_parameter_value()
        # self.pubInitialPose()     # for initial pose
        # print(response.result)
        # if response.result==0:
        #     self.pubInitialPose(0.0, 0.0)     # for initial pose
        #     print('set initial pose complete')
        #     return True
        # else:
        #     print("Set Initial Pose Failed")
        # return False
    

def main(args=None):
    rclpy.init(args=args)

    nav_service_server = NavHandler()

    rclpy.spin(nav_service_server)
    print('spin out')

    nav_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
