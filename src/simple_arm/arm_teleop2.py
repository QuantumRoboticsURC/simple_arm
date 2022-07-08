#SIMPLE ARM: roslaunch simple_arm arm_teleop.launch joystick_serial_dev:=/dev/input/js0
#!/usr/bin/python

import rospy
import subprocess

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

class ArmTeleop:
   def __init__(self):
	# Las cajas de los brazos (harmonic drive) se mueven sentido horario

	# Declaracion Grado 4 ID 1 -- PRUEBA CON BOTONES
	# 2048 = 1 vuelta del motor ; 35:1 harmonic drive ; 2048*35/360 = 199.1111111 por grado real ; limite de 90 = 17920
        self.joint4_speed = rospy.get_param('~joint4_speed', 199.1111111) # rotations of change per button press (1 grados)
        self.joint4_maxrotation = rospy.get_param('~joint4_maxrotation', 17920) # max angle of servo rotation (90 grados)
        self.joint4_minrotation = rospy.get_param('~joint4_minrotation', 0) # min angle of servo rotation
        self.joint4_position = 0 # home servo position
	# Declaracion Grado 3 ID 0 -- PRUEBA CON AXIS [CONTROL DE VELOCIDAD EXTERNO]
	# 2048 = 1 vuelta del motor ; 50:1 harmonic drive ; 2048*50/360 = 284.4444444 por grado real ; limite de 90 = 25600
        self.joint3_speed = rospy.get_param('~joint3_speed', 284.4444444) # rotations of change per button press (1 grados)
        self.joint3_maxrotation = rospy.get_param('~joint3_maxrotation', 25600) # max angle of servo rotation
        self.joint3_minrotation = rospy.get_param('~joint3_minrotation', 0) # min angle of servo rotation
        self.joint3_position = 0 # home servo position
	# Declaracion Grado 1 ID ? -- PRUEBA CON AXIS [CONTROL DE VELOCIDAD INTERNO]
	# 2048 = 1 vuelta del motor ; 50:1 harmonic drive ; 2048*50/360 = 284.4444444 por grado real ; limite de 90 = 25600
	self.joint1_speed = rospy.get_param('~joint1_speed', 284.4444444) # rotations of change per button press (1 grados)
        self.joint1_maxrotation = rospy.get_param('~joint1_maxrotation', 25600) # max angle of servo rotation
        self.joint1_minrotation = rospy.get_param('~joint1_minrotation', 0) # min angle of servo rotation
        self.joint1_position = 0 # home servo position
	# VARIABLES funcion joints
	self.twist = Twist()
	# Publicacion
	self.joint4_pub = rospy.Publisher("arm/joint4", Float32, queue_size=1)
	self.joint3_pub = rospy.Publisher("arm/joint3", Float32, queue_size=1)
	self.joint1_pub = rospy.Publisher("arm/joint1", Float32, queue_size=1)
	self.arm_twist_pub = rospy.Publisher("arm_twist", Twist, queue_size=1)
	# Suscripcion
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)
	rospy.Subscriber("joints", Twist, self.joints)

   def on_joy(self, data):
        # Grado 4 ID 1 Caja 35:1
      	# backward es decrementar la posicion
      	# forward es incrementar la posicion
      	if data.buttons[1]:
	   self.joint4_position -= self.joint4_speed
	   if self.joint4_position < self.joint4_minrotation:
	      self.joint4_position = self.joint4_minrotation
    	if data.buttons[2]:
	   self.joint4_position += self.joint4_speed
	   if self.joint4_position > self.joint4_maxrotation:
	      self.joint4_position = self.joint4_maxrotation
    	if data.buttons[3]:
	   self.joint4_position = 0
	self.joint4_pub.publish(self.joint4_position)

	# Grado 3 ID 0 Caja 50:1
      	if data.axes[1]<-0.1:
	   self.joint3_position -= self.joint3_speed
	   if self.joint3_position < self.joint3_minrotation:
	      self.joint3_position = self.joint3_minrotation
    	if data.axes[1]>0.1:
	   self.joint3_position += self.joint3_speed
	   if self.joint3_position > self.joint3_maxrotation:
	      self.joint3_position = self.joint3_maxrotation
    	if data.buttons[3]:
	   self.joint3_position = 0
	self.joint3_pub.publish(self.joint3_position)

	# Grado 1
      	if data.axes[1]<-0.1:
	   self.joint1_position -= self.joint1_speed*abs(data.axes[4])
	   if self.joint1_position < self.joint1_minrotation:
	      self.joint1_position = self.joint1_minrotation
    	if data.axes[1]>0.1:
	   self.joint1_position += self.joint1_speed*abs(data.axes[4])
	   if self.joint1_position > self.joint1_maxrotation:
	      self.joint1_position = self.joint1_maxrotation
    	if data.buttons[3]:
	   self.joint1_position = 0
	self.joint1_pub.publish(self.joint1_position)

	self.joints(self.twist)

   def joints(self,twist):
  	# Publish Twist
	twist.linear.x = self.joint1_position
	twist.linear.y = self.joint3_position
	twist.linear.z = self.joint4_position
	twist.angular.x = self.joint1_position/self.joint1_speed
	twist.angular.y = self.joint3_position/self.joint3_speed
	twist.angular.z = self.joint4_position/self.joint4_speed
	self.arm_twist_pub.publish(twist)

def main():
    rospy.init_node("arm_teleop2")
    controller = ArmTeleop()
    rospy.spin()
