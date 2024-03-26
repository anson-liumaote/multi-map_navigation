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

from example_interfaces.action import Fibonacci

import time

# ros2 action send_goal nav_action example_interfaces/action/Fibonacci "{order: 1}"

class NavHandler(Node):
    def __init__(self):
        super().__init__('nav_act_node')
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
        
        self._action_server = ActionServer(self,Fibonacci,'nav_action',self.execute_callback)
        self.counter = 0

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.mapServer()
        # self.setPose(161.94, 73.79, 0.0, 0.036)  # for navigation pose
        self.setPose(155.9, 78.3, 0.0, 0.036)  # for navigation pose

        self.runNavigation()
        self.switchMap()
        self.setPose(161.94, 73.79, 0.0, 0.036)  # for navigation pose

        self.runNavigation()
        self.switchMap()
        goal_handle.succeed()
        result = Fibonacci.Result()
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [int(self.counter)]
        self.counter = self.counter+1
        goal_handle.publish_feedback(feedback_msg)
        result.sequence = [1]
        return result

    def mapServer(self):
        self.get_logger().info('Running map server')
        id_list = [1]
        return id_list

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
        self.req.map_url = '/home/csl/Desktop/test_load_map_ws/src/load_map/maps/tsmc_b1_map.yaml'      # change map url
        print('loading map:', self.req.map_url)
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        time.sleep(2)
        return future.result()
    
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
        print(response.result)
        if response.result==0:
            # self.pubInitialPose(0.0, 0.0)     # for initial pose
            print('set initial pose complete')
            return True
        else:
            print("Set Initial Pose Failed")
        return False
    

def main(args=None):
    rclpy.init(args=args)

    nav_action_server = NavHandler()

    rclpy.spin(nav_action_server)
    print('spin out')

    nav_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
