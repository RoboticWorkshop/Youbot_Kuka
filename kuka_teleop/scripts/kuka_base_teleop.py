#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist

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

moveBindings = {
	'a':( 0.0 ,0.1  , 0.),
	'd':( 0.0 ,-0.1 , 0.),
	'w':( 0.1 ,0.0  , 0.),
	'x':(-0.1 ,0.0  , 0.),
	'A':( 0.0 ,0.5  , 0.),
	'D':( 0.0 ,-0.5 , 0.),
	'W':( 0.5 ,0.0  , 0.),
	'X':(-0.5 ,0.0  , 0.),
	'q':( 0.0 ,0.0  , 0.1),
	'e':( 0.0 ,0.0  ,-0.1),
	'Q':( 0.0 ,0.0  , 0.5),
	'E':( 0.0 ,0.0  ,-0.5),	
}

x_spd = 0.0
y_spd = 0.0
ang_spd = 0.0

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('kuka_base_controller')
	base_publisher = rospy.Publisher('/youbot/cmd_vel', Twist, queue_size = 10)
	mov = Twist()
	try:
		print(msg)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x_spd   = x_spd + moveBindings[key][0]
				y_spd   = y_spd   + moveBindings[key][1]
				ang_spd = ang_spd + moveBindings[key][2]
				mov.linear.x  = x_spd   
				mov.linear.y  = y_spd   
				mov.angular.z = ang_spd
			else:
				x_spd = 0.; y_spd = 0.; ang_spd = 0.
				mov.linear.x  = 0.
				mov.linear.y  = 0.
				mov.angular.z = 0.
				if (key == '\x03'):
					break	
			base_publisher.publish(mov)
		
	except Exception as e:
		print(e)

	finally:
		mov.linear.x  = 0.
		mov.linear.y  = 0.
		mov.angular.z = 0.		
		
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
