#!/usr/bin/env python3

# # '''
# # This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
# # This node publishes and subsribes the following topics:

# # 		PUBLICATIONS			SUBSCRIPTIONS
# # 		/drone_command			/whycon/poses
# # 		/pid_error				/throttle_pid
# # 								/pitch_pid
# # 								/roll_pid
					
# # Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
# # CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
# # '''

# # # Importing the required libraries

# # from rc_msgs.msg import RCMessage
# # from rc_msgs.srv import CommandBool
# # from geometry_msgs.msg import PoseArray
# # from pid_msg.msg import PIDTune, PIDError
# # import rclpy
# # from rclpy.node import Node
# # import time
# # import scipy.signal

# # CMD = [[], [], []]

# # MIN_ROLL = 1000
# # # BASE_ROLL = 1500
# # MAX_ROLL = 1700
# # SUM_ERROR_ROLL_LIMIT = 5000

# # MIN_PITCH = 1000
# # # BASE_PITCH = 1500
# # MAX_PITCH = 1700
# # SUM_ERROR_PITCH_LIMIT = 5000

# # MIN_THROTTLE = 1000
# # # BASE_THROTTLE = 1500
# # MAX_THROTTLE = 1700
# # SUM_ERROR_THROTTLE_LIMIT = 5000

# # class Swift_Pico(Node):
# # 	def __init__(self):
# # 		super().__init__('pico_controller')  # initializing ros node with name pico_controller

# # 		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
# # 		# [x,y,z]
# # 		self.drone_position = [0.0, 0.0, 0.0, 0.0]

# # 		# [x_setpoint, y_setpoint, z_setpoint]
# # 		self.setpoint = [0, 0, 26, 0]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

# # 		# Declaring a cmd of message type swift_msgs and initializing values
# # 		self.cmd = RCMessage()
# # 		self.cmd.rc_roll = 1500
# # 		self.cmd.rc_pitch = 1500
# # 		self.cmd.rc_yaw = 1500
# # 		self.cmd.rc_throttle = 1500

# # 		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
# # 		#after tuning and computing corresponding PID parameters, change the parameters

# # 		self.Kp = [0, 0, 0, 0]
# # 		self.Ki = [0, 0, 0, 0]
# # 		self.Kd = [0, 0, 0, 0]

# # 		#-----------------------Add other required variables for pid here ----------------------------------------------

# # 		self.error = [0.0,0.0,0.0,0.0]
# # 		self.d_error = [0.0,0.0,0.0,0.0]
# # 		self.p_error = [0.0, 0.0, 0.0,0.0]
# # 		self.iterm = [0.0,0.0,0.0,0.0]
# # 		self.previous_error = [0.0, 0.0, 0.0, 0.0]
# # 		self.out= [0.0,0.0,0.0,0.0]
# # 		self.position_history = {0: [], 1: [], 2: []}  # History for [x, y, z]
# # 		self.history_size = 20
# # 		self.flag = 0

# # 		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
# # 		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
# # 		#																	You can change the upper limit and lower limit accordingly. 
# # 		#----------------------------------------------------------------------------------------------------------

# # 		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
# # 		self.sample_time = 0.060  # in seconds
		
# # 		# Publishing /drone_command, /pid_error
# # 		self.command_pub = self.create_publisher(RCMessage, '/drone/rc_command', 10)
# # 		# self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

# # 		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------

# # 		self.pid_out_error_pub = self.create_publisher(PIDError, '/pid_out_error', 10)
# # 		self.piself_d_error_throttle_pub = self.create_publisher(PIDError, '/throttle_error', 10)
# # 		self.piself_d_error_pitch_pub = self.create_publisher(PIDError, '/pitch_error', 10)
# # 		self.piself_d_error_roll_pub = self.create_publisher(PIDError, '/roll_error', 10)
	

# # 		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
# # 		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
# # 		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)

# # 		#------------------------Add other ROS Subscribers here-----------------------------------------------------
# # 		self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
# # 		self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)


# # 		#arm/disarm service client
# # 		self.cli = self.create_client(CommandBool, "/drone/cmd/arming")
# # 		while not self.cli.wait_for_service(timeout_sec=1.0):
# # 			self.get_logger().info('Arming service not available, waiting again,,,,')
# # 		self.req = CommandBool.Request()

# # 		future = self.send_request() # ARMING THE DRONE
# # 		rclpy.spin_until_future_complete(self, future)
# # 		response = future.result()
# # 		self.get_logger().info(response.data)

# # 		# Create a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)


# #     #drone arm/disarm function 
# # 	def send_request(self):
# # 		self.req.value = True
# # 		return self.cli.call_async(self.req)

# # 	def whycon_callback(self, msg):
# # 		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
# # 		self.get_logger().info("bogos binted")
# # 		self.drone_position[0] = msg.poses[0].position.x
# # 		self.drone_position[1] = msg.poses[0].position.y
# # 		self.drone_position[2] = msg.poses[0].position.z
# # 		# print(self.drone_position[2])			if self.cmd.rc_roll > 2000:     #checking range i.e. bet 1000 and 2000
# # 		self.get_logger().info("bogos binted again")
# # 		self.pid()

# # 		#---------------------------------------------------------------------------------------------------------------
	
# # 	def altitude_set_pid(self, alt):
# # 		self.Kp[2] = alt.kp * 0.3 
# # 		self.Ki[2] = alt.ki * 0.0002
# # 		self.Kd[2] = alt.kd * 0.6

# # 	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------

# # 	def pitch_set_pid(self, pitch):
# # 		self.Kp[1] = pitch.kp * 0.3	
# # 		self.Ki[1] = pitch.ki * 0.0008
# # 		self.Kd[1] = pitch.kd * 0.6
# # 	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
# # 	def roll_set_pid(self, roll):
# # 		self.Kp[0] = roll.kp * 0.3  
# # 		self.Ki[0] = roll.ki * 0.0008
# # 		self.Kd[0] = roll.kd * 0.6
		

# # 	def limit_pid(self, throttle):
# # 		if throttle > 2000:
# # 			throttle = 2000
# # 		elif throttle < 1000:
# # 			throttle = 1000
# # 		return throttle
	
# # 	def limit_i_term_throttle(self, iterm):
# # 			if iterm > 2500.0:
# # 				iterm = 2500.0
# # 			elif iterm < -2500.0:
# # 				iterm = -2500.0
# # 			return iterm   

# # 	def limit_i_term_roll(self, iterm):
# # 			if iterm > 500.0:
# # 				iterm = 500.0
# # 			elif iterm < -500.0:
# # 				iterm = -500.0
# # 			return iterm   

# # 	# def limit_i_term_pitch(self, iterm):
# # 	# 		if iterm > 500.0:
# # 	# 			iterm = 2500.0
# # 	# 		elif iterm < -2500.0:
# # 	# 			iterm = -2500.0
# # 	# 		return iterm   

# # 	#----------------------------------------------------------------------------------------------------------------------

# # 	def publish_filtered_data(self, roll, pitch, throttle):

# # 		self.cmd.rc_throttle = int(throttle)
# # 		self.cmd.rc_roll = int(roll)
# # 		self.cmd.rc_pitch = int(pitch)
# # 		self.cmd.rc_yaw = int(1500)


# # 		# BUTTERWORTH FILTER low pass filter
# # 		span = 15
# # 		for index, val in enumerate([roll, pitch, throttle]):
# # 			CMD[index].append(val)
# # 			if len(CMD[index]) == span:
# # 				CMD[index].pop(0)
# # 			if len(CMD[index]) != span-1:
# # 				return
# # 			order = 3 # determining order 
# # 			fs = 30 # to keep in order same as hz topic runs
# # 			fc = 4 
# # 			nyq = 0.5 * fs
# # 			wc = fc / nyq
# # 			b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
# # 			filtered_signal = scipy.signal.lfilter(b, a, CMD[index])
# # 			if index == 0:
# # 				self.cmd.rc_roll = int(filtered_signal[-1])
# # 			elif index == 1:
# # 				self.cmd.rc_pitch = int(filtered_signal[-1])
# # 			elif index == 2:
# # 				self.cmd.rc_throttle = int(filtered_signal[-1])
    
# # 			print("THROTTLE", self.cmd.rc_throttle)
# # 			print("roll", self.cmd.rc_roll)
# # 			print("pitch", self.cmd.rc_pitch)

# # 			print("THROTTLE2", self.cmd.rc_throttle)
# # 			print("roll2", self.cmd.rc_roll)
# # 			print("pitch2", self.cmd.rc_pitch)

# # 		print("publising")
# # 		print(self.cmd.rc_throttle)
# # 		print(self.cmd.rc_pitch)
# # 		print(self.cmd._rc_roll)
# # 		# self.command_pub.publish(self.cmd)
# # 		print("publised")	
  		
# # 		return [self.cmd.rc_roll, self.cmd.rc_pitch, self.cmd.rc_throttle, 0]
    
# #     	# self.cmd.rc_throttle = 1400 + int(throttle)
# # 		# self.cmd.rc_roll = 1500 + int(roll)
# # 		# self.cmd.rc_pitch = 1500 + int(pitch)
# # 		# self.cmd.rc_yaw = int(1500)


# # 	def pid(self):

# # 		self.error[0] = self.drone_position[0] - self.setpoint[0] #Error in roll
# # 		self.error[1] = self.drone_position[1] - self.setpoint[1] #Error in pitch
# # 		self.error[2] = self.drone_position[2] - self.setpoint[2] #Error in throttle
# # 		self.error[3] = self.drone_position[3] - self.setpoint[3] #Error in throttle
		
# # 		self.p_error[0] = self.error[0] * self.Kp[0] #error in roll
# # 		self.p_error[1] = self.error[1] * self.Kp[1] #error in pitch
# # 		self.p_error[2] = self.error[2] * self.Kp[2] #error in throttle
# # 		self.p_error[3] = self.error[3] * self.Kp[3] #error in throttle
  
# # 		print("kp kp", self.Kp[1], self.Kp[2])

# # 		raw_d_error = [0.0] * 4
# # 		raw_d_error[0] = self.Kd[0] * (self.error[0] - self.previous_error[0])  # Roll
# # 		raw_d_error[1] = self.Kd[1] * (self.error[1] - self.previous_error[1])  # Pitch
# # 		raw_d_error[2] = self.Kd[2] * (self.error[2] - self.previous_error[2])  # Throttle
# # 		# raw_d_error[3] = self.Kd[3] * (self.error[3] - self.previous_error[3])  # Yaw

# # 		# Low-pass filter for Throttle D-term
# # 		alpha = 0.5  # Smoothing factor (adjustable)
# # 		if not hasattr(self, 'filtered_d_throttle'):
# # 			self.filtered_d_throttle = 0.0  # Initialize filter state
# # 			self.filtered_d_roll = 0.0  # Initialize filter state
# # 			self.filtered_d_pitch = 0.0  # Initialize filter state

# # 		self.filtered_d_throttle = alpha * raw_d_error[2] + (1 - alpha) * self.filtered_d_throttle
# # 		self.filtered_d_roll = alpha * raw_d_error[0] + (1 - alpha) * self.filtered_d_roll
# # 		self.filtered_d_pitch = alpha * raw_d_error[1] + (1 - alpha) * self.filtered_d_pitch

# # 		self.d_error[2] = self.filtered_d_throttle
# # 		self.d_error[0] = self.filtered_d_roll
# # 		self.d_error[1] = self.filtered_d_pitch

# # 		self.iterm[0] = self.limit_i_term_roll(self.iterm[0] + self.error[0])
# # 		self.iterm[1] = self.limit_i_term_roll(self.iterm[1] + self.error[1])
# # 		self.iterm[2] = self.limit_i_term_throttle(self.iterm[2] + self.error[2])

# # 		self.Kp[3] = 2 #YAW KP
# # 		# Output calculation
# # 		self.out[0] = self.p_error[0] + self.Ki[0] * self.iterm[0] + self.d_error[0]  # Roll
# # 		self.out[1] = self.p_error[1] + self.Ki[1] * self.iterm[1] + self.d_error[1]  # Pitch
# # 		self.out[2] = self.p_error[2] + self.Ki[2] * self.iterm[2] + self.d_error[2]  # Throttle
# # 		# self.out[3] = self.p_error[3] # Yaw

# # 		# out_filtered = []
  
# # 		# out_filtered = self.publish_filtered_data(roll = self.out[0], pitch = self.out[1], throttle = self.out[2])

# # 		# trimmed_throttle = self.limit_pid(1400 + out_filtered[2])
# # 		# trimmed_roll = self.limit_pid(1500 + out_filtered[0])
# # 		# trimmed_pitch = self.limit_pid(1500 + out_filtered[1])
# # 		# trimmed_yaw = self.limit_pid(1500 + out_filtered[3])
  
# # 		trimmed_throttle = self.limit_pid(1400 + self.out[2])
# # 		trimmed_roll = self.limit_pid(1500 - self.out[0])
# # 		trimmed_pitch = self.limit_pid(1500 + self.out[1])
# # 		trimmed_yaw = self.limit_pid(1500 + self.out[3])
  
  
# # 		self.cmd = RCMessage()
# # 		self.cmd.rc_roll = int(trimmed_roll)
# # 		self.cmd.rc_yaw = int(trimmed_yaw)
# # 		self.cmd.rc_pitch = int(trimmed_pitch)
# # 		self.cmd.rc_throttle = int(trimmed_throttle)
# # 		print("idhr aa gya")
# # 		print(self.out[2])
# # 		print(self.out[0])
# # 		print(self.out[1])	
# # 		print(self.cmd.rc_throttle)
# # 		print(self.cmd.rc_roll)
# # 		print(self.cmd.rc_pitch)
  
  
  
# # 		self.command_pub.publish(self.cmd)

# # 		for i in range(3):
# # 			self.previous_error[i] = self.error[i]		


# # 		self.pid_out_error = PIDError()
# # 		self.pid_out_error.roll_error = self.error[0]
# # 		self.pid_out_error.pitch_error = self.error[1]
# # 		self.pid_out_error.yaw_error = 0.0
# # 		self.pid_out_error.throttle_error = self.error[2]

# # 		self.pid_out_error_pub.publish(self.pid_out_error)
  



# # 		print("P error in throttle = ",self.p_error[2])
# # 		print("D error in throttle = ",self.d_error[2])
# # 		print("I error in thorttle = ",self.iterm[2] )
# # 		print("-----------------------------------------------/n")
# # 		print("P error in roll = ",self.p_error[0] )
# # 		print("D error in roll = ",self.d_error[0] )
# # 		print("I error in roll = ",self.iterm[0] )
# # 		print("-----------------------------------------------/n")
# # 		print("P error in pitch = ",self.p_error[1] )
# # 		print("D error in pitch = ",self.d_error[1])
# # 		print("I error in pitch = ",self.iterm[1])
# # 		print("-----------------------------------------------/n")
# # 		print("final ROLL velocity = ",self.out[0])
# # 		print("final PITCH velocity = ",self.out[1])
# # 		print("final THROTTLE velocty = ",self.out[2])

# # 		time.sleep(self.sample_time)




# # def main(args=None):
# # 	rclpy.init(args=args)
# # 	swift_pico = Swift_Pico()

# # 	try:
# # 		rclpy.spin(swift_pico)
# # 	except KeyboardInterrupt:
# # 		swift_pico.get_logger().info('KeyboardInterrupt, shutting down.\n')
# # 	finally:
# # 		swift_pico.destroy_node()
# # 		rclpy.shutdown()

# # if __name__ == '__main__':
# # 	main()



# '''
# This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
# This node publishes and subsribes the following topics:

# 		PUBLICATIONS			SUBSCRIPTIONS
# 		/drone_command			/whycon/poses
# 		/pid_error				/throttle_pid
# 								/pitch_pid
# 								/roll_pid
					
# Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
# CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
# '''

# # Importing the required libraries
# from rc_msgs.msg import RCMessage
# from rc_msgs.srv import CommandBool
# from geometry_msgs.msg import PoseArray
# from pid_msg.msg import PIDTune, PIDError
# import rclpy
# from rclpy.node import Node
# import time
# # from scipy.signal import butter, filtfilt


# class Swift_Pico(Node):
#     def __init__(self):
#         super().__init__('pico_controller')  # Initializing ROS node with name pico_controller

#         self.drone_position = [0.0, 0.0, 0.0]
#         self.setpoint = [0, 0, 26]

#         self.cmd = RCMessage()
#         self.cmd.rc_roll = 1500
#         self.cmd.rc_pitch = 1500
#         self.cmd.rc_yaw = 1500
#         self.cmd.rc_throttle = 1500

#         self.Kp = [0, 0, 0]
#         self.Ki = [0, 0, 0]
#         self.Kd = [0, 0, 0]

#         self.error = [0.0, 0.0, 0.0]
#         self.d_error = [0.0, 0.0, 0.0]
#         self.p_error = [0.0, 0.0, 0.0]
#         self.iterm = [0.0, 0.0, 0.0]
#         self.previous_error = [0.0, 0.0, 0.0]
#         self.out = [0.0, 0.0, 0.0]

#         self.sample_time = 0.060

#         self.command_pub = self.create_publisher(RCMessage, '/drone/rc_command', 10)
#         self.pid_out_error_pub = self.create_publisher(PIDError, '/pid_out_error', 10)

#         self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
#         self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
#         self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
#         self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

#         self.cli = self.create_client(CommandBool, "/drone/cmd/arming")
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Arming service not available, waiting again...')
#         self.req = CommandBool.Request()

#         future = self.send_request()  # ARMING THE DRONE
#         rclpy.spin_until_future_complete(self, future)
#         response = future.result()
#         self.get_logger().info(response.data)


#     def send_request(self):
#         self.req.value = True
#         return self.cli.call_async(self.req)

#     def whycon_callback(self, msg):
#         self.drone_position[0] = msg.poses[0].position.x
#         self.drone_position[1] = msg.poses[0].position.y
#         self.drone_position[2] = msg.poses[0].position.z
#         self.pid()

#     def altitude_set_pid(self, alt):
#         self.Kp[2] = alt.kp * 0.3
#         self.Ki[2] = alt.ki * 0.0008
#         self.Kd[2] = alt.kd * 0.6

#     def pitch_set_pid(self, pitch):
#         self.Kp[1] = pitch.kp * 0.3
#         self.Ki[1] = pitch.ki * 0.008
#         self.Kd[1] = pitch.kd * 0.6

#     def roll_set_pid(self, roll):
#         self.Kp[0] = roll.kp * 0.03
#         self.Ki[0] = roll.ki * 0.008
#         self.Kd[0] = roll.kd * 0.6

#     def limit_pid(self, value):
#         if value > 2000:
#             return 2000
#         elif value < 1000:
#             return 1000
#         return value

#     def limit_i_term(self, iterm):
#         if iterm > 5000.0:
#             return 5000.0
#         elif iterm < -5000.0:
#             return -5000.0
#         return iterm

#     def pid(self):
#         self.error[0] = self.drone_position[0] - self.setpoint[0]  # Roll error
#         self.error[1] = self.drone_position[1] - self.setpoint[1]  # Pitch error
#         self.error[2] = self.drone_position[2] - self.setpoint[2]  # Throttle error

#         for i in range(3):
#             self.p_error[i] = self.error[i] * self.Kp[i]
#             self.d_error[i] = self.Kd[i] * (self.error[i] - self.previous_error[i])
#             self.iterm[i] = self.limit_i_term(self.iterm[i] + self.error[i])
#             self.out[i] = self.p_error[i] + self.Ki[i] * self.iterm[i] + self.d_error[i]

#         self.cmd.rc_roll = int(self.limit_pid(1520 - self.out[0]))
#         self.cmd.rc_pitch = int(self.limit_pid(1520 + self.out[1]))
#         self.cmd.rc_throttle = int(self.limit_pid(1430 + self.out[2]))
#         self.cmd.rc_yaw = 1520

#         self.command_pub.publish(self.cmd)

#         print("THROTTLE", self.cmd.rc_throttle)
#         print("roll", self.cmd.rc_roll)
#         print("pitch", self.cmd.rc_pitch)

#         for i in range(3):
#             self.previous_error[i] = self.error[i]

#         pid_error_msg = PIDError()
#         pid_error_msg.roll_error = self.error[0]
#         pid_error_msg.pitch_error = self.error[1]
#         pid_error_msg.throttle_error = self.error[2]
#         self.pid_out_error_pub.publish(pid_error_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = Swift_Pico()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# Importing the required libraries
from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node
import scipy.signal


class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')  # Initializing ROS node with name pico_controller
        self.drone_position = [0.0, 0.0, 0.0]
        self.setpoint = [0, 0, 26]

        self.cmd = RCMessage()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        self.Kp = [0, 0, 0]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 0]

        self.error = [0.0, 0.0, 0.0]
        self.d_error = [0.0, 0.0, 0.0]
        self.p_error = [0.0, 0.0, 0.0]
        self.iterm = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]
        self.out = [0.0, 0.0, 0.0]

        self.sample_time = 0.060

        self.command_pub = self.create_publisher(RCMessage, '/drone/rc_command', 10)
        self.pid_out_error_pub = self.create_publisher(PIDError, '/pid_out_error', 10)

        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

        self.cli = self.create_client(CommandBool, "/drone/cmd/arming")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting again...')
        self.req = CommandBool.Request()

        future = self.send_request()  # ARMING THE DRONE
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(str(response.data))

    def publish_filtered_data(self, roll, pitch, throttle):
        self.cmd.rc_throttle = int(throttle)
        self.cmd.rc_roll = int(roll)
        self.cmd.rc_pitch = int(pitch)
        self.cmd.rc_yaw = 1500

        # BUTTERWORTH FILTER low pass filter
        span = 15
        CMD = [[], [], []]
        for index, val in enumerate([roll, pitch, throttle]):
            CMD[index].append(val)
            if len(CMD[index]) == span:
                CMD[index].pop(0)
            if len(CMD[index]) != span - 1:
                order = 3  # determining order
                fs = 30  # to keep in order same as hz topic runs
                fc = 4.5
                nyq = 0.5 * fs
                wc = fc / nyq
                b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
                filtered_signal = scipy.signal.lfilter(b, a, CMD[index])
            if index == 0:
                self.cmd.rc_roll = int(filtered_signal[-1])
            elif index == 1:
                self.cmd.rc_pitch = int(filtered_signal[-1])
            elif index == 2:
                self.cmd.rc_throttle = int(filtered_signal[-1])

        final = [self.cmd.rc_roll, self.cmd.rc_pitch, self.cmd.rc_throttle, 1500]

        self.cmd.rc_roll = int(self.limit_pid(1500 - final[0]))
        self.cmd.rc_pitch = int(self.limit_pid(1500 + final[1]))
        self.cmd.rc_throttle = int(self.limit_pid(1460 + final[2]))
        self.cmd.rc_yaw = 1520

        print("Roll", self.cmd.rc_roll)
        print("Pitch", self.cmd.rc_pitch)
        print("Throttle", self.cmd.rc_throttle)
        self.command_pub.publish(self.cmd)

    def send_request(self):
        self.req.value = True
        return self.cli.call_async(self.req)

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        self.pid()

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.3
        self.Ki[2] = alt.ki * 0.0008
        self.Kd[2] = alt.kd * 0.6

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.kp * 0.3
        self.Ki[1] = pitch.ki * 0.0008
        self.Kd[1] = pitch.kd * 0.6

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp * 0.03
        self.Ki[0] = roll.ki * 0.0008
        self.Kd[0] = roll.kd * 0.6

    def limit_pid(self, value):
        if value > 2000:
            return 2000
        elif value < 1000:
            return 1000
        return value

    def limit_i_term(self, iterm):
        if iterm > 10000.0:
            return 10000.0
        elif iterm < -10000.0:
            return -10000.0
        return iterm

    def pid(self):
        self.error[0] = self.drone_position[0] - self.setpoint[0]  # Roll error
        self.error[1] = self.drone_position[1] - self.setpoint[1]  # Pitch error
        self.error[2] = self.drone_position[2] - self.setpoint[2]  # Throttle error

        for i in range(3):
            self.p_error[i] = self.error[i] * self.Kp[i]
            self.d_error[i] = self.Kd[i] * (self.error[i] - self.previous_error[i])
            self.iterm[i] = self.limit_i_term(self.iterm[i] + self.error[i])
            self.out[i] = self.p_error[i] + self.Ki[i] * self.iterm[i] + self.d_error[i]

        self.publish_filtered_data(self.out[0], self.out[1], self.out[2])

        for i in range(3):
            self.previous_error[i] = self.error[i]

        pid_error_msg = PIDError()
        pid_error_msg.roll_error = self.error[0]
        pid_error_msg.pitch_error = self.error[1]
        pid_error_msg.throttle_error = self.error[2]
        self.pid_out_error_pub.publish(pid_error_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Swift_Pico()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()