#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from waypoint_navigation.srv import GetFirstPath  
from waypoint_navigation.srv import GetSecondPath 
from waypoint_navigation.action import NavToWaypoint  
from functools import partial
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool, Int32MultiArray
                
class PathActionClient(Node):
    def __init__(self):
        super().__init__('path_action_client')
        
        self.count = 0
        self.flag = True  # Start with flag as True for the first iteration
        self.waypoints = []
        self.waypoints2 = []
        self.counter = 0
        
        # Subscriber to update flag
        self.create_subscription(Bool, "/path_req", self.flag_update, 1)
        self.create_subscription(Int32MultiArray, '/random_points', self.points_callback, 1)

        # Action client
        self._action_client = ActionClient(self, NavToWaypoint, 'navigate_to_pose')
        
        # Service client
        self.first_path_client = self.create_client(GetFirstPath, 'plannedpath1')

        # Wait for the service to be available
        while not self.first_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GetFirstPath service...')
        
        # Request the first path
        self.request_first_path()

    def points_callback(self, msg):
        # self.waypoints = msg.data[1], msg.data[2], 27.0
        self.start = (msg.data[0], msg.data[1], 27.0)
        self.end = (msg.data[2], msg.data[3], 27.0)


    def flag_update(self, msg):
        # Update the flag based on the received message
        self.flag = msg.data
        # self.get_logger().info(f"Flag updated to: {self.flag}")
        
        # Trigger processing immediately if flag becomes True
        if self.flag:
            self.process_next_waypoint(is_first_path=True)


    def request_first_path(self):
        self.count = 0
        request = GetFirstPath.Request()
        request.get_path1 = True
        self.future_first_path = self.first_path_client.call_async(request)
        self.future_first_path.add_done_callback(self.handle_first_path_response)

    def handle_first_path_response(self, future):
        try:
            response = future.result()
            self.waypoints2 = response
            self.get_logger().info('Received First Path.')
            # Start path processing for the first time
            self.process_path(self.waypoints2, is_first_path=True)
        except Exception as e:
            self.get_logger().error(f'Error in first path response: {e}')

    def process_path(self, pose_array, is_first_path):
        self.get_logger().info("Processing path points...")

        if self.counter >= 1:
            print("array cleared")
            self.waypoints.clear()
            print(self.waypoints)

        for pose in pose_array.plannedpath1.poses:
            print(self.count)
            self.get_logger().info(f"Waypoint - x: {pose.position.x}, y: {pose.position.y}, z: {pose.position.z}")

            self.waypoints.append(pose)  # Store the entire pose, not just coordinates
            # print(self.waypoints)

        # Process the first waypoint

        print(self.waypoints)
        # self.waypoints.append()
        self.counter = self.counter + 1
        self.process_next_waypoint(is_first_path)



    def process_next_waypoint(self, is_first_path):
        if self.count >= len(self.waypoints):
            self.get_logger().info("All waypoints have been processed.")

            if is_first_path and self.count >= len(self.waypoints) - 1:
                self.get_logger().info('First path completed. Requesting second path...')
                # self.count = 0
                self.request_first_path()
            elif not is_first_path and self.count >= len(self.waypoints) - 1:
                self.get_logger().info('Second path completed.')
                return
        
        # Goal message setup for the current waypoint
        goal_msg = NavToWaypoint.Goal()
        goal_msg.waypoint = self.waypoints[self.count]
        goal_msg.which_path = 1 if is_first_path else 2
        
        self.get_logger().info(f"Sending goal for waypoint {self.count}.")
        self._action_client.wait_for_server()
        self.send_goal(goal_msg, is_first_path)

        # Reset flag after sending the goal
        self.flag = False
        self.count = self.count + 1

    def send_goal(self, goal_msg, is_first_path):
        self.get_logger().info("Sending goal to action server...")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(partial(self.goal_response_callback, is_first_path=is_first_path))

    def goal_response_callback(self, future, is_first_path):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        
        self._get_result_future.add_done_callback(partial(self.get_result_callback, is_first_path=is_first_path))

    def get_result_callback(self, future, is_first_path):
        result = future.result().result

        if result.path_reached:
            self.get_logger().info(f"Waypoint {self.count} reached.")


def main():
    rclpy.init()
    action_client = PathActionClient()
    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
