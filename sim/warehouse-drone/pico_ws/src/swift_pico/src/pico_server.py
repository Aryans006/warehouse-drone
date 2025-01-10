#!/usr/bin/env python3

import time
import math
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

#import the action
from waypoint_navigation.action import NavToWaypoint

#pico control specific libraries
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
from nav_msgs.msg import Odometry

class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        self.time_inside_sphere = 0
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.duration = 0


        self.drone_position = [0.0, 0.0, 0.0, 0.0]
        self.setpoint = [0, 0, 26, 0] 
        self.dtime = 0

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        #Kp, Ki and Kd values here
        self.Kp = [0, 0, 0, 0]
        self.Ki = [0, 0, 0, 0]
        self.Kd = [0, 0, 0, 0]
		#-----------------------Add other required variables for pid here ----------------------------------------------
        self.error = [0,0,0, 0]
        self.d_error = [0.0,0.0,0.0,0.0, 0.0]
        self.p_error = [0, 0, 0, 0]
        self.iterm = [0,0,0,0]
        self.previous_error = [0, 0, 0, 0]
        self.out = [0,0,0,0]
        
        #[roll, pitch, throttle, yaw]

        # self.Kp[2] = 55 * 0.8  # throttle
        # self.Ki[2] = 11 * 0.000059
        # self.Kd[2] = 1000 * 0.9

        # self.Kp[1] = 35 * 0.5  # pitch
        # self.Ki[1] = 15 * 0.0000591
        # self.Kd[1] = 1000  * 0.9

        # self.Kp[0] = 35 * 0.5  # roll
        # self.Ki[0] = 20 * 0.0000651
        # self.Kd[0] = 1000 * 0.9

        # self.Kp[1] = 20 * 0.5  # pitch
        # self.Ki[1] = 3 * 0.0000591
        # self.Kd[1] = 400  * 0.9

        # self.Kp[0] = 15 * 0.5  # roll
        # self.Ki[0] = 4 * 0.0000651
        # self.Kd[0] = 500 * 0.9





        self.pid_error = PIDError()

        self.sample_time = 0.060

        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_out_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        #Add other subscribers here
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)
        self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10)

        #create an action server for the action 'NavToWaypoint'. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        #action name should 'waypoint_navigation'.
        #include the action_callback_group in the action server. Refer to executors in ROS 2 concepts
        self.waypoint_navigation_=ActionServer(self,NavToWaypoint,"waypoint_navigation",self.execute_callback)
        self.get_logger().info("Action server started")
        self.arm()

        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.pid_callback_group)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)


    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)


    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        #Set the remaining co-ordinates of the drone from msg
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

        self.dtime = msg.header.stamp.sec

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.8 
        self.Ki[2] = alt.ki * 0.000052
        self.Kd[2] = alt.kd * 0.9

    #Define callback function like altitide_set_pid to tune pitch, roll

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.kp * 0.5		 
        self.Ki[1] = pitch.ki * 0.000057
        self.Kd[1] = pitch.kd * 0.9
        
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp * 0.5
        self.Ki[0] = roll.ki * 0.000065
        self.Kd[0] = roll.kd * 0.17

    def limit_pid(self, throttle):
        if throttle > 2000:
            throttle = 2000
        elif throttle < 1000:
            throttle = 1000
        return throttle
    
    def limit_i_term(self, iterm):
        if iterm > 64.0:
            iterm = 64.0
        elif iterm < -64.0:
            iterm = -64
        return iterm    

    def odometry_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)
        self.yaw_deg = math.degrees(yaw)
        self.drone_position[3] = self.yaw_deg	

     # def pid(self):

    #     #write your PID algorithm here. This time write equations for throttle, pitch, roll and yaw. 
    #     #Follow the steps from task 1b.

    #     self.error[0] = self.drone_position[0] - self.setpoint[0] #Error in roll
    #     self.error[1] = self.drone_position[1] - self.setpoint[1] #Error in pitch
    #     self.error[2] = self.drone_position[2] - self.setpoint[2] #Error in throttle
    #     self.error[3] = self.drone_position[3] - self.setpoint[3] #Error in yaw

    #     print("self.error: ",self.error[2])
    #     print(self.drone_position[2])

    #     self.p_error[0] = self.error[0] * self.Kp[0] #error in roll
    #     self.p_error[1] = self.error[1] * self.Kp[1] #error in pitch
    #     self.p_error[2] = self.error[2] * self.Kp[2] #error in throttle
    #     self.p_error[3] = self.error[3] * self.Kp[3] #error in yaw

    #     self.d_error[0] = self.Kd[0]*(self.error[0] - self.previous_error[0]) #error in roll
    #     self.d_error[1] = self.Kd[1]*(self.error[1] - self.previous_error[1]) #error in pitch
    #     self.d_error[2] = self.Kd[2]*(self.error[2] - self.previous_error[2]) #error in throttle
    #     self.d_error[3] = self.Kd[3]*(self.error[3] - self.previous_error[3]) #error in yaw
        
    #     self.iterm[0] = self.limit_i_term(self.iterm[0]+ self.error[0]) #error in roll
    #     self.iterm[1] = self.limit_i_term(self.iterm[1] + self.error[1]) #error in pitch
    #     self.iterm[2] = self.limit_i_term(self.iterm[2] + self.error[2]) #error in throttle
    #     self.iterm[3] = self.limit_i_term(self.iterm[3] + self.error[3]) #error in yaw
        
    #     self.out[0] = (self.Ki[0]*(self.iterm[0])) + self.d_error[0] + self.p_error[0]#final error in roll
    #     self.out[1] = (self.Ki[1]*(self.iterm[1])) + self.d_error[1] + self.p_error[1]#final error in pitch
    #     self.out[2] = (self.Ki[2]*(self.iterm[2])) + self.d_error[2] + self.p_error[2]#final error in throttle
    #     self.out[3] = (self.Ki[3]*(self.iterm[3])) + self.d_error[3] + self.p_error[3]#final error in yaw
        
    #     trimmed_throttle = self.limit_pid(1532 + self.out[2])
    #     trimmed_roll = self.limit_pid(1500 - self.out[0])
    #     trimmed_pitch = self.limit_pid(1500 + self.out[1])
    #     trimmed_yaw = self.limit_pid(1500 + self.out[3])  # okeji
        
    #     self.cmd.rc_roll = int(trimmed_roll)
    #     self.cmd.rc_yaw = 1500
    #     self.cmd.rc_pitch = int(trimmed_pitch)
    #     self.cmd.rc_throttle = int(trimmed_throttle)
    #     self.cmd.rc_aux4 = 2000
    #     self.command_pub.publish(self.cmd)
        
    #     pookie = self.error[2]
    #     self.previous_error[2] = float(pookie)
        
    #     pookie_pitch = self.error[1]
    #     self.previous_error[1] = float(pookie_pitch)
        
    #     pookie_roll = self.error[0]
    #     self.previous_error[0] = float(pookie_roll)

    #     pookie_yaw = self.error[3]
    #     self.previous_error[3] = float(pookie_yaw)
        
    #     print("P error in throttle = ",self.p_error[2])
    #     print("D error in throttle = ",self.d_error[2])
    #     print("I error in throttle = ",self.iterm[2] )
    #     print("-----------------------------------------------/n")
    #     print("P error in roll = ",self.p_error[0] )
    #     print("D error in roll = ",self.d_error[0] )
    #     print("I error in roll = ",self.iterm[0] )
    #     print("-----------------------------------------------/n")
    #     print("P error in pitch = ",self.p_error[1] )
    #     print("D error in pitch = ",self.d_error[1])
    #     print("I error in pitch = ",self.iterm[1])
    #     print("-----------------------------------------------/n")
    #     print("P error in yaw = ",self.p_error[3] )
    #     print("D error in yaw = ",self.d_error[3])
    #     print("I error in yaw = ",self.iterm[3])        
    #     print("-----------------------------------------------/n") 
    #     print("final ROLL velocity = ",self.out[0])
    #     print("final PITCH velocity = ",self.out[1])
    #     print("final THROTTLE velocty = ",self.out[2])
    #     print("final YAW velocty = ",self.out[3])
       
    #     self.pid_error.roll_error = self.error[0]
    #     self.pid_error.pitch_error = self.error[1]
    #     self.pid_error.yaw_error = self.error[3]
    #     self.pid_error.throttle_error = self.error[2]
    #     self.pid_out_error_pub.publish(self.pid_error)

    def pid(self):
        # Errors
        self.error[0] = self.drone_position[0] - self.setpoint[0]  # Roll
        self.error[1] = self.drone_position[1] - self.setpoint[1]  # Pitch
        self.error[2] = self.drone_position[2] - self.setpoint[2]  # Throttle
        self.error[3] = self.drone_position[3] - self.setpoint[3]  # Yaw

        # Proportional Errors
        self.p_error[0] = self.error[0] * self.Kp[0]  # Roll
        self.p_error[1] = self.error[1] * self.Kp[1]  # Pitch
        self.p_error[2] = self.error[2] * self.Kp[2]  # Throttle
        self.p_error[3] = self.error[3] * self.Kp[3]  # Yaw

        # Derivative Errors (Unfiltered)
        raw_d_error = [0.0] * 4
        raw_d_error[0] = self.Kd[0] * (self.error[0] - self.previous_error[0])  # Roll
        raw_d_error[1] = self.Kd[1] * (self.error[1] - self.previous_error[1])  # Pitch
        raw_d_error[2] = self.Kd[2] * (self.error[2] - self.previous_error[2])  # Throttle
        raw_d_error[3] = self.Kd[3] * (self.error[3] - self.previous_error[3])  # Yaw

        # Low-pass filter for Throttle D-term
        alpha = 0.68  # Smoothing factor (adjustable)
        if not hasattr(self, 'filtered_d_throttle'):
            self.filtered_d_throttle = 0.0  # Initialize filter state
            self.filtered_d_roll = 0.0  # Initialize filter state
            self.filtered_d_pitch = 0.0  # Initialize filter state

        self.filtered_d_throttle = alpha * raw_d_error[2] + (1 - alpha) * self.filtered_d_throttle
        self.filtered_d_roll = alpha * raw_d_error[0] + (1 - alpha) * self.filtered_d_roll
        self.filtered_d_pitch = alpha * raw_d_error[1] + (1 - alpha) * self.filtered_d_pitch

        self.d_error[2] = self.filtered_d_throttle
        self.d_error[0] = self.filtered_d_roll
        self.d_error[1] = self.filtered_d_pitch

        # # Update D-terms for Roll, Pitch, and Yaw (unchanged)
        # self.d_error[0] = raw_d_error[0]
        # self.d_error[1] = raw_d_error[1]
        # self.d_error[3] = raw_d_error[3]

        # Integral Terms (with limiting)
        for i in range(4):
            self.iterm[i] = self.limit_i_term(self.iterm[i] + self.error[i])

        # Output calculation
        self.out[0] = self.p_error[0] + self.Ki[0] * self.iterm[0] + self.d_error[0]  # Roll
        self.out[1] = self.p_error[1] + self.Ki[1] * self.iterm[1] + self.d_error[1]  # Pitch
        self.out[2] = self.p_error[2] + self.Ki[2] * self.iterm[2] + self.d_error[2]  # Throttle
        self.out[3] = self.p_error[3] + self.Ki[3] * self.iterm[3] + self.d_error[3]  # Yaw

        # Limit throttle output only
        trimmed_throttle = self.limit_pid(1532 + self.out[2])
        trimmed_roll = self.limit_pid(1500 - self.out[0])
        trimmed_pitch = self.limit_pid(1500 + self.out[1])
        trimmed_yaw = self.limit_pid(1500 + self.out[3])


        # Publish the commands
        self.cmd.rc_roll = int(trimmed_roll)
        self.cmd.rc_pitch = int(trimmed_pitch)
        self.cmd.rc_yaw = int(trimmed_yaw)
        self.cmd.rc_throttle = int(trimmed_throttle)
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

        # Update previous errors for next iteration
        for i in range(4):
            self.previous_error[i] = self.error[i]

        # Debugging
        # print(f"Filtered D error (Throttle): {self.filtered_d_throttle}")
        # print(f"Final Throttle Command: {trimmed_throttle}")

        # print("P error in throttle = ",self.p_error[2])
        # print("D error in throttle = ",self.d_error[2])
        # print("I error in throttle = ",self.iterm[2] )
        # print("-----------------------------------------------/n")
        # print("P error in roll = ",self.p_error[0] )
        # print("D error in roll = ",self.d_error[0] )
        # print("I error in roll = ",self.iterm[0] )
        # print("-----------------------------------------------/n")
        # print("P error in pitch = ",self.p_error[1] )
        # print("D error in pitch = ",self.d_error[1])
        # print("I error in pitch = ",self.iterm[1])
        # print("-----------------------------------------------/n")
        # print("P error in yaw = ",self.p_error[3] )
        # print("D error in yaw = ",self.d_error[3])
        # print("I error in yaw = ",self.iterm[3])        
        # print("-----------------------------------------------/n") 
        # print("final ROLL velocity = ",self.out[0])
        # print("final PITCH velocity = ",self.out[1])
        # print("final THROTTLE velocty = ",self.out[2])
        # print("final YAW velocty = ",self.out[3])
       
        self.pid_error.roll_error = self.error[0]
        self.pid_error.pitch_error = self.error[1]
        self.pid_error.yaw_error = self.error[3]
        self.pid_error.throttle_error = self.error[2]
        self.pid_out_error_pub.publish(self.pid_error)



    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        self.setpoint[0] = goal_handle.request.waypoint.position.x
        self.setpoint[1] = goal_handle.request.waypoint.position.y
        self.setpoint[2] = goal_handle.request.waypoint.position.z
        self.get_logger().info(f'New Waypoint Set: {self.setpoint}')
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0
        self.duration = self.dtime

        #create a NavToWaypoint feedback object. Refer to Writing an action server and client (Python) in ROS 2 tutorials.
        feedback_msg=NavToWaypoint.Feedback()

        #--------The script given below checks whether you are hovering at each of the waypoints(goals) for max of 3s---------#
        # This will help you to analyse the drone behaviour and help you to tune the PID better.

        while True:
            feedback_msg.current_waypoint.pose.position.x = self.drone_position[0]
            feedback_msg.current_waypoint.pose.position.y = self.drone_position[1]
            feedback_msg.current_waypoint.pose.position.z = self.drone_position[2]
            feedback_msg.current_waypoint.header.stamp.sec = self.max_time_inside_sphere

            goal_handle.publish_feedback(feedback_msg)
            # time.sleep(1)

            drone_is_in_sphere = self.is_drone_in_sphere(self.drone_position, goal_handle, 0.4) #the value '0.4' is the error range in the whycon coordinates that will be used for grading. 
            #You can use greater values initially and then move towards the value '0.4'. This will help you to check whether your waypoint navigation is working properly. 

            if not drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        pass
            
            elif drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        self.point_in_sphere_start_time = self.dtime
                        self.get_logger().info('Drone in sphere for 1st time')
 
            elif drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
                        self.get_logger().info('Drone in sphere')
                        print(self.time_inside_sphere)
                        
                             
            elif not drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.get_logger().info('Drone out of sphere')
                        self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
                        self.point_in_sphere_start_time = None

            if self.time_inside_sphere > self.max_time_inside_sphere:
                 self.max_time_inside_sphere = self.time_inside_sphere

            if self.max_time_inside_sphere >= 3.7:
                 break
                        

        goal_handle.succeed()
    
        #create a NavToWaypoint result object. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        result=NavToWaypoint.Result()
        result.hov_time = self.dtime - self.duration
        return result

    def is_drone_in_sphere(self, drone_pos, sphere_center, radius):
        return (
            (drone_pos[0] - sphere_center.request.waypoint.position.x) ** 2
            + (drone_pos[1] - sphere_center.request.waypoint.position.y) ** 2
            + (drone_pos[2] - sphere_center.request.waypoint.position.z) ** 2
        ) <= radius**2


def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)
    
    try:
         executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
         waypoint_server.destroy_node()
         rclpy.shutdown()


if __name__ == '__main__':
    main()
