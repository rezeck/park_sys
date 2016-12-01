#!/usr/bin/env python

import rospy
import tf
import math
from individuo import Individuo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

class Solver(object):
	"""docstring for Solver"""
	def __init__(self, num_gen = 10, num_pop = 10):
		super(Solver, self).__init__()
		self.laser = None
		self.odom = None
		self.dist = 0.0
		self.goal_x = rospy.get_param("/park_sys/goal_x")
		self.goal_y = rospy.get_param("/park_sys/goal_y")
		self.goal_theta = rospy.get_param("/park_sys/goal_theta")
		print "[Status]: Park position ", self.goal_x, self.goal_y, self.goal_theta
		self.num_gen = num_gen
		self.num_pop = num_pop
		print "[Status]: Generation size", num_gen, "and Population size ", num_pop
		print "[Status]: Creating population..."
		self.pop = []
		for i in range(self.num_pop):
			ind = Individuo(rand=True)
			self.pop.append(ind)

	def odomCallback(self, data):
		self.odom = data
		quaternion = (
			data.pose.pose.orientation.x,
			data.pose.pose.orientation.y,
			data.pose.pose.orientation.z,
			data.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		dx = abs(self.odom.pose.pose.position.x - self.goal_x)
		dy = abs(self.odom.pose.pose.position.y - self.goal_y)
		dtheta = abs(yaw - self.goal_theta)
		self.dist = math.sqrt(dx*dx + dy*dy + dtheta*dtheta)

	def checkCollision(self):
		return rospy.get_param("/crash")

	def reset_position(self):
		self.reset_stage()

	def fitness(self, collision_penality=2.0):
		if self.checkCollision():
			return self.dist*collision_penality
		else:
			return self.dist

	def test(self, ind):
		self.reset_position()
		t = Twist()
		for command in ind.getVelocidades():
			if self.checkCollision():
				break
			t.linear.x = command[0]*0.5
			t.angular.z = command[1]*1.0
			self.pub_cmd_vel.publish(t)
			self.rate.sleep()
		return self.fitness()

	def run(self):
		rospy.init_node('park_sys', anonymous=False)
		print "[Status]: Running node park_sys..."
		self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	
		rospy.Subscriber("/base_pose_ground_truth", Odometry, self.odomCallback)
		self.reset_stage = rospy.ServiceProxy("/reset_positions", Empty)

		self.rate = rospy.Rate(1) # 10hz
		
		while not rospy.is_shutdown():
			for g in range(self.num_gen):
				print "################################"
				print "[Status]: Generation #", g
				print "################################"
				for p in range(self.num_pop):
					print "[Status]: Testing individo ", p, "\n", self.pop[p].getVelocidades()
					self.pop[p].fitness = self.test(self.pop[p])
					print "[Status]: Fitness ", self.pop[p].fitness
					print "################################"
			self.rate.sleep()

if __name__ == '__main__':
	try:
		solver = Solver()
		solver.run()
	except rospy.ROSInterruptException:
		pass