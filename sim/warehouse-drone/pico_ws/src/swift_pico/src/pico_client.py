#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

#import the action and service
from rclpy.action.client import ClientGoalHandle
from waypoint_navigation.srv import GetWaypoints
from waypoint_navigation.action import NavToWaypoint 
from functools import partial


class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')
        self.goals = []
        self.goal_index = 0
        #create an action client for the action 'NavToWaypoint'. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        #action name should 'waypoint_navigation'.
        self.waypoint_navigation = ActionClient(self, NavToWaypoint, "waypoint_navigation")

        #create a client for the service 'GetWaypoints'. Refer to Writing a simple service and client (Python) in ROS 2 tutorials
        #service name should be 'waypoints'

        self.waypoints = self.create_client(GetWaypoints, 'waypoints')
        
        while not self.waypoints.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        #create a request object for GetWaypoints service.
        
        self.request_waypoints = GetWaypoints.Request()

    
    ###action client functions

    def send_goal(self, waypoint):

        #create a NavToWaypoint goal object.
        goal_msg = NavToWaypoint.Goal()

        goal_msg.waypoint.position.x = waypoint[0]
        goal_msg.waypoint.position.y = waypoint[1]
        goal_msg.waypoint.position.z = waypoint[2]

        #create a method waits for the action server to be available.
        self.waypoint_navigation.wait_for_server()

        self.send_goal_future = self.waypoint_navigation.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)    
        self.send_goal_future.add_done_callback(self.goal_response_callback)    

    def goal_response_callback(self, future):
        #complete the goal_response_callback. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        goal_handle = future.result()
        if goal_handle.accepted:
            print("bogos ne send krdiya6")
            goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        #complete the missing line
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.hov_time))

        self.goal_index += 1

        if self.goal_index < len(self.goals):
            self.send_goal(self.goals[self.goal_index])
        else:
            self.get_logger().info('All waypoints have been reached successfully')      

    def feedback_callback(self, feedback_msg):

        #complete the missing line
        feedback = feedback_msg.feedback
        x = feedback.current_waypoint.pose.position.x
        y = feedback.current_waypoint.pose.position.y
        z = feedback.current_waypoint.pose.position.z
        t = feedback.current_waypoint.header.stamp.sec
        self.get_logger().info(f'Received feedback! The current whycon position is: {x}, {y}, {z}')
        self.get_logger().info(f'Max time inside sphere: {t}')


    #service client functions

    def send_request(self):
        #  complete send_request method, which will send the request and return a future
        test = True
        self.request_waypoints.get_waypoints = test
 
        future = self.waypoints.call_async(self.request_waypoints)
        # future.add_done_callback(self.receive_goals)
        
        return future
    
    def receive_goals(self):
        future = self.send_request()  
        #write a statement to execute the service until the future is complete
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        self.get_logger().info('Waypoints received by the action client')

        for pose in response.waypoints.poses:
            waypoints = [pose.position.x, pose.position.y, pose.position.z]
            self.goals.append(waypoints)
            self.get_logger().info(f'Waypoints: {waypoints}')

        self.send_goal(self.goals[0])
    

def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WayPointClient()
    waypoint_client.receive_goals()
    print("bogos binted")

    try:
        rclpy.spin(waypoint_client)
    except KeyboardInterrupt:
        waypoint_client.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_client.destroy_node()
        rclpy.shutdown()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
        