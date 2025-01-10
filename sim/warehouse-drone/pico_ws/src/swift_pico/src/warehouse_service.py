#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from waypoint_navigation.srv import GetFirstPath
from waypoint_navigation.srv import GetSecondPath
from env import Env
# from scripts import self

"""
Bidirectional_a_star 2D
@author: huiming zhou
"""

import math
import heapq


class BidirectionalAStar:
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = Env("pico_ws/src/swift_pico/src/image.webp")  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN_fore = []  # OPEN set for forward searching
        self.OPEN_back = []  # OPEN set for backward searching
        self.CLOSED_fore = []  # CLOSED set for forward
        self.CLOSED_back = []  # CLOSED set for backward
        self.PARENT_fore = dict()  # recorded parent for forward
        self.PARENT_back = dict()  # recorded parent for backward
        self.g_fore = dict()  # cost to come for forward
        self.g_back = dict()  # cost to come for backward

    def init(self):
        """
        initialize parameters
        """

        self.g_fore[self.s_start] = 0.0
        self.g_fore[self.s_goal] = math.inf
        self.g_back[self.s_goal] = 0.0
        self.g_back[self.s_start] = math.inf
        self.PARENT_fore[self.s_start] = self.s_start
        self.PARENT_back[self.s_goal] = self.s_goal
        heapq.heappush(self.OPEN_fore,
                       (self.f_value_fore(self.s_start), self.s_start))
        heapq.heappush(self.OPEN_back,
                       (self.f_value_back(self.s_goal), self.s_goal))

    def searching(self):
        """
        Bidirectional A*
        :return: connected path, visited order of forward, visited order of backward
        """

        self.init()
        s_meet = self.s_start

        while self.OPEN_fore and self.OPEN_back:
            # solve foreward-search
            _, s_fore = heapq.heappop(self.OPEN_fore)

            if s_fore in self.PARENT_back:
                s_meet = s_fore
                break

            self.CLOSED_fore.append(s_fore)

            for s_n in self.get_neighbor(s_fore):
                new_cost = self.g_fore[s_fore] + self.cost(s_fore, s_n)

                if s_n not in self.g_fore:
                    self.g_fore[s_n] = math.inf

                if new_cost < self.g_fore[s_n]:
                    self.g_fore[s_n] = new_cost
                    self.PARENT_fore[s_n] = s_fore
                    heapq.heappush(self.OPEN_fore,
                                   (self.f_value_fore(s_n), s_n))

            # solve backward-search
            _, s_back = heapq.heappop(self.OPEN_back)

            if s_back in self.PARENT_fore:
                s_meet = s_back
                break

            self.CLOSED_back.append(s_back)

            for s_n in self.get_neighbor(s_back):
                new_cost = self.g_back[s_back] + self.cost(s_back, s_n)

                if s_n not in self.g_back:
                    self.g_back[s_n] = math.inf

                if new_cost < self.g_back[s_n]:
                    self.g_back[s_n] = new_cost
                    self.PARENT_back[s_n] = s_back
                    heapq.heappush(self.OPEN_back,
                                   (self.f_value_back(s_n), s_n))

        return self.extract_path(s_meet), self.CLOSED_fore, self.CLOSED_back

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def extract_path(self, s_meet):
        """
        extract path from start and goal
        :param s_meet: meet point of bi-direction a*
        :return: path
        """

        # extract path for foreward part
        path_fore = [s_meet]
        s = s_meet

        while True:
            s = self.PARENT_fore[s]
            path_fore.append(s)
            if s == self.s_start:
                break

        # extract path for backward part
        path_back = []
        s = s_meet

        while True:
            s = self.PARENT_back[s]
            path_back.append(s)
            if s == self.s_goal:
                break

        return list(reversed(path_fore)) + list(path_back)

    def f_value_fore(self, s):
        """
        forward searching: f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g_fore[s] + self.h(s, self.s_goal)

    def f_value_back(self, s):
        """
        backward searching: f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g_back[s] + self.h(s, self.s_start)

    def h(self, s, goal):
        """
        Calculate heuristic value.
        :param s: current node (state)
        :param goal: goal node (state)
        :return: heuristic value
        """

        heuristic_type = self.heuristic_type

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            # print(f"Collision detected at {s_start} or {s_end}")
            return True
        # print("OBSTALCES", self.obs)
        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

class WayPoints(Node):

    def __init__(self):
        super().__init__('waypoints_service')
        self.counter = 0
        self.counter2 = 0
        self.drone_position = (500, 500)
        self.start = (630, 120)
        self.end = (120.0, 130.0)
        # self.path_array = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.flag = True
        # self.path_array[0] = self.drone_position
        # self.path_array[1] = self.start
        # self.path_array[2] = self.end


        self.waypoints = []

        self.subscrpition = self.create_subscription(Int32MultiArray, '/package_loc', self.points_callback, 1)
        self.new_way = self.create_publisher(Int32MultiArray, '/point_published', 10)
        # self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.srv = self.create_service(GetFirstPath, 'plannedpath1', self.path1_callback)
        print("service chaluuuu")
        # self.srv = self.create_service(GetSecondPath, 'plannedpath2', self.path2_callback)


    def pixel_to_whycon(self, img):
        print("img", img)
        goal_x= 0.02537*img - 12.66
        return goal_x
    
    def pixel_to_whycon2(self, img):
        goal_x= 0.02534*img - 12.57
        return goal_x



    def whycon_callback(self, whycon):

        while a == True:
            self.get_logger().info("bogos binted")
            self.drone_position[0] = whycon.poses[0].position.x
            self.drone_position[1] = whycon.poses[0].position.y
            self.drone_position[2] = whycon.poses[0].position.z
            a = False


    def points_callback(self, msg):

        if self.counter2 < 3:
            # print(self.counter2)
            point = (msg.data[0], msg.data[1]) 
            if (msg.data[0], msg.data[1]) not in self.waypoints: # Extract the x and y coordinates.
                self.waypoints.append(point)        # Append the new point to waypoints array.
                self.counter2 += 1
                print(self.counter2)                 # Increment counter for each new point.
                self.get_logger().info(f"Point added: {point}")
            
        else:
            while self.counter2 == 3:
                print(self.counter2)
                print("popino")
                print(self.waypoints)
                temp = self.waypoints[1]
                self.waypoints[1] = self.waypoints[2]
                self.waypoints[2] = temp
                self.counter2 = self.counter2 + 1
                print(self.waypoints)

                break
            print("popino 2")
            data = []
            print()
            if len(self.waypoints) > 2 and self.flag == True:

                for i in range(0, len(self.waypoints)):
                    for j in range(0,2):
                        data.append(self.waypoints[i][j])
                        msg.data = data
                        print(data)
                        # self.flag = False
                
                temp = msg.data[1]
                msg.data[1] = msg.data[2]
                msg.data[2] = temp
                self.new_way.publish(msg)
                print("PULBISEDDDD")
            
        

        # print(self.flag)


                    # print(self.flag)


         
    def path1_callback(self, request, response):

        if self.counter == 0:
            pathing = (self.waypoints[0], self.waypoints[1])
            # bastar = BidirectionalAStar((self.drone_position), (self.waypoints[0], self.waypoints[1]), "euclidean")
            print(pathing[0])
            bastar = BidirectionalAStar(self.drone_position[:2], pathing[0], "euclidean")

            path, visited_fore, visited_back = bastar.searching()
            self.counter = 0
            # path.insert(0,(500,500))
            print("this is path1")

            if request.get_path1:
                    # Populate poses based on the path
                response.plannedpath1.poses = [
                    Pose(
                        position=Point(
                            x=self.pixel_to_whycon(path[i][0]),
                            y=self.pixel_to_whycon2(path[i][1]),
                            z=27.0
                        ),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    )
                    for i in range(0, len(path), 20)
                ]

                # Append your final pose
                custom_pose = Pose(
                    position=Point(
                        x=self.pixel_to_whycon(self.waypoints[0][0]),
                        y=self.pixel_to_whycon2(self.waypoints[0][1]),
                        z=27.0
                    ),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )

                custom_pose_2 = Pose(
                    position=Point(
                        x=self.pixel_to_whycon(self.drone_position[0]),
                        y=self.pixel_to_whycon2(self.drone_position[1]),
                        z=27.0
                    ),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )
                # response.plannedpath1.poses.insert(0, custom_pose_2)
                response.plannedpath1.poses.append(custom_pose)
                self.get_logger().info("Incoming request for path")
                print(response.plannedpath1.poses)
                self.counter += 1
                return response
            else:
                self.get_logger().info("Request rejected")

        elif self.counter == 1:
            pathing1 = (self.waypoints[0][0], self.waypoints[0][1])
            pathing2 = (self.waypoints[1][0], self.waypoints[1][1])
            bastar = BidirectionalAStar(pathing1, pathing2, "euclidean")
            path, visited_fore, visited_back = bastar.searching()
            print("path2")
            # path.insert(0,(self.start[:2]))
            # path.append(self.start)
            # print(self.start)
            # print("this is path2", path)

            if request.get_path1:
                    # Populate poses based on the path
                response.plannedpath1.poses = [
                    Pose(
                        position=Point(
                            x=self.pixel_to_whycon(path[i][0]),
                            y=self.pixel_to_whycon2(path[i][1]),
                            z=27.0
                        ),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    )
                    for i in range(0, len(path), 20)
                ]

                # Append your specific Pose at the end
                custom_pose_3 = Pose(
                    position=Point(
                        x=self.pixel_to_whycon(self.waypoints[0][0]),
                        y=self.pixel_to_whycon2(self.waypoints[0][1]),
                        z=27.0
                    ),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )

                custom_pose_4 = Pose(
                    position=Point(
                        x=self.pixel_to_whycon(self.waypoints[1][0]),
                        y=self.pixel_to_whycon2(self.waypoints[1][1]),
                        z=27.0
                    ),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )

                # response.plannedpath1.poses.insert(0, custom_pose_3)
                response.plannedpath1.poses.append(custom_pose_4)

                self.get_logger().info("Incoming request for path")
                print(response.plannedpath1.poses)
                self.counter = self.counter + 1
                return response
            else:
                self.get_logger().info("Request rejected")

        elif self.counter == 2:
            pathing3 = (self.waypoints[1][0], self.waypoints[1][1])
            pathing4 = (self.waypoints[2][0], self.waypoints[2][1])
            bastar = BidirectionalAStar(pathing3, pathing4, "euclidean")
            path, visited_fore, visited_back = bastar.searching()

            # path.insert(0,(self.start[:2]))
            print("this is path3")

            if request.get_path1:
                    # Populate poses based on the path
                response.plannedpath1.poses = [
                    Pose(
                        position=Point(
                            x=self.pixel_to_whycon(path[i][0]),
                            y=self.pixel_to_whycon2(path[i][1]),
                            z=27.0
                        ),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    )
                    for i in range(0, len(path), 20)
                ]

                # Append your specific Pose at the end
                custom_pose_5 = Pose(
                    position=Point(
                        x=self.pixel_to_whycon(self.waypoints[1][0]),
                        y=self.pixel_to_whycon2(self.waypoints[1][1]),
                        z=27.0
                    ),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )

                custom_pose_6 = Pose(
                    position=Point(
                        x=self.pixel_to_whycon(self.waypoints[2][0]),
                        y=self.pixel_to_whycon2(self.waypoints[2][1]),
                        z=27.0
                    ),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )

                # response.plannedpath1.poses.insert(0, custom_pose_5)
                response.plannedpath1.poses.append(custom_pose_6)
                self.get_logger().info("Incoming request for path")
                print(response.plannedpath1.poses)
                self.counter += 1
                return response
            
            else:
                self.get_logger().info("Request rejected")




#     def path2_callback(self, request, response):

#         # print("path array 0", self.path_array[0][:2])
#         # print("path array 1", self.path_array[1][:2])

#         bastar = BidirectionalAStar(self.path_array[0], self.path_array[1], "euclidean")
#         path, visited_fore, visited_back = bastar.searching()

#         print("this is path", path)
#         if request.get_path2 == True :
#             response.plannedpath2.poses = [
#                 # Block to define each pose
#                 Pose(
#                     position= Point(
#                         x=self.pixel_to_whycon(path[i][0]),  
#                         y=self.pixel_to_whycon(path[i][1]),  
#                         z=27.0 
#                     ),
#                     orientation= Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#     )
#         for i in range(0, len(path), 4)
# ]
#             self.get_logger().info("Incoming request for path")
#             return response
#         else:
#             self.get_logger().info("Request rejected")

def main():
    rclpy.init()
    waypoints = WayPoints()

    try:
        rclpy.spin(waypoints)
    except KeyboardInterrupt:
        waypoints.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoints.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        

        