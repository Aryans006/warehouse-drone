#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error				/throttle_pid
								/pitch_pid
								/roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node
import time

class Swift_Pico(Node):
    def __init__(self):
        self.flag = 0
        super().__init__('pico_controller')
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
        self.d_error = [0.0, 0.0, 0.0, 0.0]
        self.p_error = [0.0, 0.0, 0.0]
        self.iterm = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]
        self.out = [0.0, 0.0, 0.0]
        self.sample_time = 0.05
        self.command_pub = self.create_publisher(RCMessage, '/drone/rc_command', 10)
        self.pid_out_error_pub = self.create_publisher(PIDError, '/pid_out_error', 10)
        self.piself_d_error_throttle_pub = self.create_publisher(PIDError, '/throttle_error', 10)
        self.piself_d_error_pitch_pub = self.create_publisher(PIDError, '/pitch_error', 10)
        self.piself_d_error_roll_pub = self.create_publisher(PIDError, '/roll_error', 10)
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)
        self.cli = self.create_client(CommandBool, "/drone/cmd/arming")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting again,,,,')
        self.req = CommandBool.Request()
        future = self.send_request()
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(response.data)

    def send_request(self):
        self.req.value = True
        return self.cli.call_async(self.req)

    def whycon_callback(self, msg):
        self.get_logger().info("bogos binted")
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        self.get_logger().info("bogos binted again")
        self.pid()
        self.flag = self.flag + 1

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.008
        self.Kd[2] = alt.kd * 0.6

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.kp * 0.3
        self.Ki[1] = pitch.ki * 0.008
        self.Kd[1] = pitch.kd * 0.6

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp * 0.03
        self.Ki[0] = roll.ki * 0.008
        self.Kd[0] = roll.kd * 0.6

    def limit_pid(self, throttle):
        if throttle > 2000:
            throttle = 2000
        elif throttle < 1000:
            throttle = 1000
        return throttle

    def pid(self):
        while True:
            self.cmd.rc_roll = 1500
            self.cmd.rc_yaw = 1500
            self.cmd.rc_pitch = 1500
            self.cmd.rc_throttle = 1500
            self.command_pub.publish(self.cmd)
            
            print("Throttle", self.cmd.rc_throttle)
            print("roll", self.cmd.rc_roll)
            print("pitch", self.cmd.rc_pitch)
            print("yaw", self.cmd.rc_yaw)


def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    try:
        rclpy.spin(swift_pico)
    except KeyboardInterrupt:
        swift_pico.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        swift_pico.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
