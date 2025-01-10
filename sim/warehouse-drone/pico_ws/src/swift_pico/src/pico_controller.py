#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/piself_d_error				/throttle_pid
								/pitch_pid
								/roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries


import rclpy.logging
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node
import time


class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # initializing ros node with name pico_controller

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0, 0, 0]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [2, 2, 27] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		# Declaring a cmd of message type swift_msgs and initializing values
		
		self.flag = 0
		
		self.cmd = SwiftMsgs()
		self.cmd.rc_roll = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_throttle = 1500

		self.pid_error = PIDError()
		self.pid_error.roll_error = 0.0
		self.pid_error.pitch_error = 0.0
		self.pid_error.yaw_error = 0.0
		self.pid_error.throttle_error = 0.0


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]
		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.error = [0,0,0]
		self.d_error = [0.0,0.0,0.0,0.0]
		self.p_error = [0, 0, 0]
		self.iterm = [0,0,0]
		self.previous_error = [0, 0, 0]
		self.out= [0,0,0]

		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
	
		self.sample_time = 0.05  # in seconds

		# Publishing /drone_command, /throttle_error, /pitch_error, /roll_error
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pid_d_error_pub = self.create_publisher(PIDError, '/pid_d_error', 10)
		self.pid_p_error_pub = self.create_publisher(PIDError, '/pid_p_error', 10)
		self.pid_i_error_pub = self.create_publisher(PIDError, '/pid_i_error', 10)
		self.pid_out_error_pub = self.create_publisher(PIDError, '/pid_out_error', 10)
		self.piself_d_error_throttle_pub = self.create_publisher(PIDError, '/throttle_error', 10)
		self.piself_d_error_pitch_pub = self.create_publisher(PIDError, '/pitch_error', 10)
		self.piself_d_error_roll_pub = self.create_publisher(PIDError, '/roll_error', 10)


		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------
	

		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
		self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
		self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

		#------------------------Add other ROS Subscribers here-----------------------------------------------------
		self.arm()  # ARMING THE DRONE

		# Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)
		

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
		self.command_pub.publish(self.cmd)  # Publishing /drone_command

	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self, whycon):
		self.get_logger().info("bogos binted")
		self.drone_position[0] = whycon.poses[0].position.x
		self.drone_position[1] = whycon.poses[0].position.y
		self.drone_position[2] = whycon.poses[0].position.z
		# print(self.drone_position[2])
		self.get_logger().info("bogos binted again")
		self.pid()
		self.flag = self.flag + 1
		
		#---------------------------------------------------------------------------------------------------------------

	# Callback function for /throttle_pid
	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp * 0.5  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.ki * 0.0005
		self.Kd[2] = alt.kd * 0.3	

	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.kp * 0.3		  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pitch.ki * 0.000052
		self.Kd[1] = pitch.kd * 0.9
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def roll_set_pid(self, roll):
		self.Kp[0] = roll.kp * 0.23  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = roll.ki * 0.000051
		self.Kd[0] = roll.kd * 0.17
	
	def limit_pid(self, throttle):
		if throttle > 2000:
			throttle = 2000
		elif throttle < 1000:
			throttle = 1000
		return throttle
		
	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):

	#  [roll pitch throttle]
			
			self.Kp[2] = 19 * 0.5  # throttle
			self.Ki[2] = 7 * 0.004
			self.Kd[2] = 350 * 0.5

			self.Kp[1] = 42 * 0.4  # pitch
			self.Ki[1] = 14 * 0.000057
			self.Kd[1] = 750 * 0.9

			self.Kp[0] = 40 * 0.23  # roll
			self.Ki[0] = 20 * 0.000065
			self.Kd[0] = 2750 * 0.17


			self.error[0] = self.drone_position[0] - self.setpoint[0] #Error in roll
			self.error[1] = self.drone_position[1] - self.setpoint[1] #Error in pitch
			self.error[2] = self.drone_position[2] - self.setpoint[2] #Error in throttle

			print("self.error ki value", self.error[2])
			print(self.drone_position[2])

			##kilculate iterm/sum error


			self.p_error[0] = self.error[0] * self.Kp[0] #error in roll
			self.p_error[1] = self.error[1] * self.Kp[1] #error in pitch
			self.p_error[2] = self.error[2] * self.Kp[2] #error in throttle

			self.d_error[0] = self.Kd[0]*(self.error[0] - self.previous_error[0]) #error in roll
			self.d_error[1] = self.Kd[1]*(self.error[1] - self.previous_error[1]) #error in pitch
			self.d_error[2] = self.Kd[2]*(self.error[2] - self.previous_error[2]) #error in throttle
			

			self.iterm[0] = (self.iterm[0]+ self.error[0]) #error in roll
			self.iterm[1] = (self.iterm[1] + self.error[1]) #error in pitch
			self.iterm[2] = (self.iterm[2] + self.error[2]) #error in throttle

			self.out[0] = (self.Ki[0]*self.iterm[0]) + self.d_error[0] + self.p_error[0]#final error in roll
			self.out[1] = (self.Ki[1]*self.iterm[1]) + self.d_error[1] + self.p_error[1]#final error in pitch
			self.out[2] = (self.Ki[2]*self.iterm[2]) + self.d_error[2] + self.p_error[2]#final error in throttle



			trimmed_throttle = self.limit_pid(1500 + self.out[2])
			trimmed_roll = self.limit_pid(1500 - self.out[0])
			trimmed_pitch = self.limit_pid(1500 + self.out[1])

			self.cmd.rc_roll = int(trimmed_roll)
			self.cmd.rc_yaw = 1500
			self.cmd.rc_pitch = int(trimmed_pitch)
			self.cmd.rc_throttle = int(trimmed_throttle)
			self.cmd.rc_aux4 = 2000
			self.command_pub.publish(self.cmd)
				
			# self.pid_error.roll_error = self.d_error[0]
			# self.pid_error.pitch_error = self.d_error[1]
			# self.pid_error.yaw_error = 0.0
			# self.pid_error.throttle_error = self.d_error[2]

			self.pid_error.roll_error = self.error[0]
			self.pid_error.pitch_error = self.error[1]
			self.pid_error.yaw_error = 0.0
			self.pid_error.throttle_error = self.error[2]

			self.pid_out_error_pub.publish(self.pid_error)

			print(self.cmd.rc_throttle)

			self.command_pub.publish(self.cmd)
			ramesh = self.error[2]
			self.previous_error[2] = float(ramesh)

			ramesh_pitch = self.error[1]
			self.previous_error[1] = float(ramesh_pitch)

			ramesh_roll = self.error[0]
			self.previous_error[0] = float(ramesh_roll)

			print("P error in throttle = ",self.p_error[2])
			print("D error in throttle = ",self.d_error[2])
			print("I error in thorttle = ",self.iterm[2] )
			print("-----------------------------------------------/n")
			print("P error in roll = ",self.p_error[0] )
			print("D error in roll = ",self.d_error[0] )
			print("I error in roll = ",self.iterm[0] )
			print("-----------------------------------------------/n")
			print("P error in pitch = ",self.p_error[1] )
			print("D error in pitch = ",self.d_error[1])
			print("I error in pitch = ",self.iterm[1])
			print("-----------------------------------------------/n")
			print("final ROLL velocity = ",self.out[0])
			print("final PITCH velocity = ",self.out[1])
			print("final THROTTLE velocty = ",self.out[2])



			time.sleep(self.sample_time)

			# break 
				
		#------------------------------------------------------------------------------------------------------------------------
			# calculate throttle error, pitch error and roll error, then publish it accordingly


def main(args=None):

	rclpy.init(args=args)
	swift_pico = Swift_Pico()
	rclpy.spin(swift_pico)
	swift_pico.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

	##todo
	##cleanup
	##PID TOONING T^T