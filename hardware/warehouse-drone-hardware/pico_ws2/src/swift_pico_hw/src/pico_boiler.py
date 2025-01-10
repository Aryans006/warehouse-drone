#!/usr/bin/env python3
'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subscribes to the following topics:

PUBLICATIONS			SUBSCRIPTIONS
/drone_command			/whycon/poses
/pid_error			/throttle_pid
			/pitch_pid
			/roll_pid
	
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries
from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node
import scipy.signal
import matplotlib.pyplot as plt
from collections import deque

class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # Initializing ROS node with name pico_controller
		self.drone_position = [0.0, 0.0, 0.0]
  
		self.alpha = 0.096
		self.estimate = None

		self.noisy = []
		self.filtered_pose = [0.0, 0.0, 0.0]
  
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

		self.estimate1 = None
		self.estimate2 = None
		self.estimate3 = None
		self.cli = self.create_client(CommandBool, "/drone/cmd/arming")
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Arming service not available, waiting again...')
		self.req = CommandBool.Request()
		future = self.send_request()  # ARMING THE DRONE
		rclpy.spin_until_future_complete(self, future)
		response = future.result()
		self.get_logger().info(str(response.data))
   
	def send_request(self):
		self.req.value = True
		return self.cli.call_async(self.req)

	def update1(self, value):
		# print("inside update", self.noisy)
		if self.estimate1 is None:

			self.estimate1 = value
		else:

			self.estimate1 = self.alpha * value + (1 - self.alpha) * self.estimate1
		return round(self.estimate1, 2)

	def update2(self, value):
		# print("inside update", self.noisy)
		if self.estimate2 is None:

			self.estimate2 = value
		else:
			self.estimate2 = self.alpha * value + (1 - self.alpha) * self.estimate2
		return round(self.estimate2, 2)

	def update3(self, value):
		# print("inside update", self.noisy)
		if self.estimate3 is None:

			self.estimate3 = value
		else:
			self.estimate3 = self.alpha * value + (1 - self.alpha) * self.estimate3
		return round(self.estimate3, 2)


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
			fc = 4
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
  
		self.cmd.rc_roll = int(self.limit_pid(1500 + final[0]))
		self.cmd.rc_pitch = int(self.limit_pid(1480 - final[1]))
		self.cmd.rc_throttle = int(self.limit_pid(1450 + final[2]))
		self.cmd.rc_yaw = 1520

		print("Roll", self.cmd.rc_roll)
		print("Pitch", self.cmd.rc_pitch)
		print("Throttle", self.cmd.rc_throttle)
		self.command_pub.publish(self.cmd)

		# return [self.cmd.rc_roll, self.cmd.rc_pitch, self.cmd.rc_throttle, 0]

	def whycon_callback(self, msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		self.noisy.append(self.drone_position)
		self.filtered_pose[2] = self.update2(self.drone_position[2])
		self.filtered_pose[0] = self.update1(self.drone_position[0])
		self.filtered_pose[1] = self.update3(self.drone_position[1])
		# print("Filtered throttle",self.filtered_pose)
		self.pid()

	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp * 0.05
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
		# self.error[0] = self.drone_position[0] - self.setpoint[0]  # Roll error
		# self.error[1] = self.drone_position[1] - self.setpoint[1]  # Pitch error
		# self.error[2] = self.drone_position[2] - self.setpoint[2]  # Throttle error
  
		self.error[0] = self.filtered_pose[0] - self.setpoint[0]  # Roll error
		self.error[1] = self.filtered_pose[1] - self.setpoint[1]  # Pitch error
		self.error[2] = self.filtered_pose[2] - self.setpoint[2]  # Throttle error

		# for i in range(3):
		# 	self.p_error[i] = self.error[i] * self.Kp[i]
		# 	self.d_error[i] = self.Kd[i] * (self.error[i] - self.previous_error[i])
		# 	self.iterm[i] = self.limit_i_term(self.iterm[i] + self.error[i])
		# 	self.out[i] = self.p_error[i] + self.Ki[i] * self.iterm[i] + self.d_error[i]

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