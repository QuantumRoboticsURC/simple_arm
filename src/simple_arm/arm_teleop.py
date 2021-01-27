#!/usr/bin/python

import rospy
import subprocess

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
class ArmTeleop:
    def __init__(self):
	# Parameters of speed and action of gripper
	# Speed values: 5, 10, 15, 20, 30
	self.gripper_speed = 5 # Default value >> 5
	self.gripper_speed_max = rospy.get_param('~gripper_speed_max', 50)/100 # max speed of grip
	self.gripper_speed_min = rospy.get_param('~gripper_speed_min', 25)/100 # min speed of grip
	self.gripper_position = 45 # Open >> 0 ; Close >> 90

        self.gripper_pub = rospy.Publisher("gripper_pos", Float32, queue_size=1)
	self.gripper_s = rospy.Publisher("gripper_speed", Float32, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)

	# Motores inician estaticos
	self.base = 0
	self.joint1 = 0 # Axes Izq (2) Der (5) -32767;32767
	self.joint2 = 0 # Axes 1
	self.joint3 = 0 # Axes 4

	self.base_pub = rospy.Publisher("arm/base", Float32, queue_size=1)
	self.joint1_pub = rospy.Publisher("joint1", Float32, queue_size=1)
	self.joint2_pub = rospy.Publisher("joint2", Float32, queue_size=1)
	self.joint3_pub = rospy.Publisher("joint3", Float32, queue_size=1)

	self.arm_twist_pub = rospy.Publisher("arm_twist", Twist, queue_size=1)

    def on_joy(self, data):
	# Arm control
	## Base
	if data.buttons[5] == 1 and data.buttons[4] == 0: # Derecha
		self.base = 0.1
	elif data.buttons[4] == 1 and data.buttons[5] == 0: # Izquierda
		self.base = -0.1
	else:
		self.base = 0

	## Joint 1
	if data.axes[2] < 1 and data.axes[5] == 1:
		# Codo arriba
		self.joint1 = (data.axes[2]-1)/-2 # Gatillo derecho - mandar 1
	elif data.axes[5] < 1 and data.axes[2] == 1:
		# Codo abajo
		self.joint1 = (data.axes[5]-1)/2 # Gatillo izquierdo - mandar -1
	else:
		self.joint1 = 0

	## Joint 2
	if data.axes[1] != 0:
		self.joint2 = data.axes[1]
	else:
		self.joint2 = 0
	
	## Joint 3
	if data.axes[4] != 0:
		# Codo arriba
		self.joint3 = data.axes[4]
	else:
		self.joint3 = 0



	# Gripper control
	if data.axes[7] < 0:
		if self.gripper_speed > self.gripper_speed_min:
			self.gripper_speed -= 0.25
		else:
			self.gripper_speed = self.gripper_speed_min
	elif data.axes[7] > 0:
		if self.gripper_speed < self.gripper_speed_max:
			self.gripper_speed += 0.25
		else:
			self.gripper_speed = self.gripper_speed_max
	if data.axes[6] > 0: # Open - right d pad
		if self.gripper_position > 0:
			self.gripper_position -= self.gripper_speed
		else:
			self.gripper_position = 0
	if data.axes[6] < 0: # Close - left d pad	
		if self.gripper_position < 90:
			self.gripper_position += self.gripper_speed
		else:
			self.gripper_position = 90
	if data.buttons[3]: # center servo position (Y button)
		self.gripper_position = 45
	self.gripper_pub.publish(self.gripper_position)
	self.gripper_s.publish(self.gripper_speed)

	self.base_pub.publish(self.base)
	self.joint1_pub.publish(self.joint1)
	self.joint2_pub.publish(self.joint2)
	self.joint3_pub.publish(self.joint3)

	# Publish Twist
	twist = Twist()
	twist.linear.x = self.joint1
	twist.linear.y = self.joint2
	twist.linear.z = self.joint3
	twist.angular.x = self.base
	self.arm_twist_pub.publish(twist)

def main():
    rospy.init_node("arm_teleop")
    controller = ArmTeleop()
    rospy.spin()
