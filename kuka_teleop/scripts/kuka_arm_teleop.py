#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import numpy as np
from brics_actuator.msg import JointPositions, JointValue, Poison

import sys, select, termios, tty

msg = """
KUKA Base Controller Program
w = increase linear x velocity by 0.1
x = decrease linear x velocity by 0.1
d = increase linear y velocity by 0.1
a = increase linear y velocity by 0.1
q = increase angular z velocity by 0.1
e = decrease angular z velocity by 0.1

CAPITAL character increase / decrease each speed by 0.5
w
"""

jointBindings = {
	'u':( 0.1,  .0,  .0,  .0,  .0),
	'i':(-0.1,  .0,  .0,  .0,  .0),
	'o':(  .0, 0.1,  .0,  .0,  .0),
	'p':(  .0,-0.1,  .0,  .0,  .0),	
	'h':(  .0,  .0, 0.1,  .0,  .0),
	'j':(  .0,  .0,-0.1,  .0,  .0),
	'k':(  .0,  .0,  .0, 0.1,  .0),
	'l':(  .0,  .0,  .0,-0.1,  .0),
	'n':(  .0,  .0,  .0,  .0, 0.1),
	'm':(  .0,  .0,  .0,  .0,-0.1),
}

check_positions = np.array([3.0, 1.0, -2.5, 1.7, 3.0])
joint_positions = np.array([3.0, 0.9, -3.5, 0.7, 3.0])

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('kuka_arm_controller')
	arm_joint_publisher = rospy.Publisher("/arm_1/arm_controller/position_command",JointPositions,queue_size=10)
	joint1 = JointValue()
	joint2 = JointValue()
	joint3 = JointValue()
	joint4 = JointValue()
	joint5 = JointValue()
	poison = Poison()
	joint_pos = JointPositions()					
	delay = rospy.Rate(10)
	try:
		print(msg)		
		joint1.joint_uri = "arm_joint_1"
		joint1.unit = "rad"
		joint1.value = check_positions[0]
				
		joint2.joint_uri = "arm_joint_2"
		joint2.unit = "rad"
		joint2.value = check_positions[1]
		
		joint3.joint_uri = "arm_joint_3"
		joint3.unit = "rad"
		joint3.value = check_positions[2]
		
		joint4.joint_uri = "arm_joint_4"
		joint4.unit = "rad"
		joint4.value = check_positions[3]		
		
		joint5.joint_uri = "arm_joint_5"
		joint5.unit = "rad"
		joint5.value = check_positions[4]		
		
		joint_pos.poisonStamp = poison
		joint_pos.positions = [joint1, joint2, joint3, joint4, joint5]

		print("Check Arm-joint")

		for i in range(0,5):
			arm_joint_publisher.publish(joint_pos)			
			delay.sleep()
		
		for i in range(0,20):
			joint1.value = joint_positions[0]
			joint2.value = joint_positions[1]
			joint3.value = joint_positions[2]
			joint4.value = joint_positions[3]
			joint5.value = joint_positions[4]
			print("Waiting process")
			delay.sleep()
		
		joint_pos.poisonStamp = poison
		joint_pos.positions = [joint1, joint2, joint3, joint4, joint5]

		print("Standby Position")
		for i in range(0,5):
			arm_joint_publisher.publish(joint_pos)			
			delay.sleep()		
			
		while not rospy.is_shutdown():
			key = getKey()
			if key in jointBindings.keys():	
				joint_positions[0] = joint_positions[0]+jointBindings[key][0]
				joint_positions[1] = joint_positions[1]+jointBindings[key][1]
				joint_positions[2] = joint_positions[2]+jointBindings[key][2]
				joint_positions[3] = joint_positions[3]+jointBindings[key][3]
				joint_positions[4] = joint_positions[4]+jointBindings[key][4]
				
			joint1.value = joint_positions[0]
			joint2.value = joint_positions[1]
			joint3.value = joint_positions[2]
			joint4.value = joint_positions[3]
			joint5.value = joint_positions[4]
			
			print(joint_positions)
			
			joint_pos.poisonStamp = poison
			joint_pos.positions = [joint1, joint2, joint3, joint4, joint5]
			for i in range(0,2):
				arm_joint_publisher.publish(joint_pos)
				print("Publish Data")
				delay.sleep()
				
			if (key == '\x03'):
				break			
						
	except Exception as e:
		print(e)

	finally:
		rospy.is_shutdown()
		print("Program Terminated")		
		
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
