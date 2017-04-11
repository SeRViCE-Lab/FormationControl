#! /usr/bin/env python

'''  ______________________________________________________________________
	/*
	*\	This code is part of the sphero_ros project.
	*/
	*\ 	Essentially, we are using the geometry_msgs/Twist msgs to send
	*/ 	velocity commands to the sphero robots
	*\
	*/
	*/	Author: Olalekan Ogunmolu
	*\	Date: April 11, 2017
	*/__________________________________________________________________________
'''

import rospy
import tqdm
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA, Float32, Bool
from multiprocessing.dummy import Pool as Thread

class MoveSphero(object):

	def __init__(self, ns =''):
		super(MoveSphero, self).__init__()
		# self.ns = ns
		# self.publishers(ns = self.ns)
		self.namespaces = [ '/sphero_rpg',
							'/sphero_ypp',
							'/sphero_www',
							'/sphero_wwb',
							'/sphero_rwp',
							'/sphero_rgo',
							'/sphero_pob',
							'/sphero_gbr']
		#make pool of workers
		self.pool_workers = Thread(4)

	def publishers(self, ns='',def_queue_size = 1):
		self.cmd_vel_pub = rospy.Publisher('%scmd_vel'%ns, Twist, queue_size = def_queue_size)
		self.color_pub = rospy.Publisher('%sset_color'%ns, ColorRGBA,  queue_size = def_queue_size)
		self.back_led_pub = rospy.Publisher('%sset_back_led'%ns, Float32,  queue_size = def_queue_size)
		self.stabilization_pub = rospy.Publisher('%sdisable_stabilization'%ns, Bool,  queue_size = def_queue_size)
		self.heading_pub = rospy.Publisher('%sset_heading'%ns, Float32,  queue_size = def_queue_size)
		self.angular_velocity_pub = rospy.Publisher('%sset_angular_velocity'%ns, Float32, queue_size = def_queue_size)

	def sphero_populate(self):
		sphero_names = self.namespaces

		# generate velocity commads
		twist_msg = Twist()
		twist_msg.linear.x = 30

		#populate the namespaces we want to use
		for i in tqdm(int(range(sphero_names)):
			publishers(self, ns=sphero_names[i])
		self.cmd_vel_pub.publish(twist_msg)

		#open namespaces in separate threads
		ns_threads = self.pool_workers.map(sphero_names, )


if __name__ == '__main__':
	try:
		rospy.init_node('sphero_command_center', anonymous=True)

		ms = MoveSphero()

		rate = rospy.Rate(10) # 10hz

		while not rospy.is_shutdown():
			ms.sphero_populate()
			rate.sleep()

	except rospy.ROSInterruptException:
		pass
