#!/usr/bin/env python

import rospy
import tf
import math
import random

from individuo import Individuo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import matplotlib.pyplot as plt


class Solver(object):
	"""docstring for Solver"""
	def __init__(self, num_gen = 200, num_pop = 20, show_output = True):
		super(Solver, self).__init__()
		self.laser = None
		self.odom = None
		self.dist = 0.0
		self.goal_x = rospy.get_param("/park_sys/goal_x")
		self.goal_y = rospy.get_param("/park_sys/goal_y")
		self.goal_theta = rospy.get_param("/park_sys/goal_theta")
		self.num_gen = num_gen
		self.num_pop = num_pop
		self.show_output = show_output
		self.statistics = []

		print "[Status]: Park position ", self.goal_x, self.goal_y, self.goal_theta
		print "[Status]: Generation size", num_gen, "and Population size ", num_pop
		print "[Status]: Creating population..."
		
		self.pop = []
		self.filhos = []
		for i in range(self.num_pop):
			ind = Individuo(rand=True, initialSize = 20)
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

	def fitness(self, collision_penality=1.2):
		if self.checkCollision():
			return self.dist*collision_penality + 1.0
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
		#initialize matplotlib
		plt.ion()

		rospy.init_node('park_sys', anonymous=False)
		print "[Status]: Running node park_sys..."
		self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	
		rospy.Subscriber("/base_pose_ground_truth", Odometry, self.odomCallback)
		self.reset_stage = rospy.ServiceProxy("/reset_positions", Empty)

		self.rate = rospy.Rate(1) # 10hz
		

		self.calculateFitnessPopulation()
		self.updateStatiscs()
		self.pop.sort()	

		while not rospy.is_shutdown():

			for g in range(1,self.num_gen):
				print "################################"
				print "[Status]: Generation #", g
				print "################################"

				self.createFilhos()
				self.calculateFitnessFilhos()
				self.selectNextGeneration()

				self.updateStatiscs()
				self.rate.sleep() 
			break

	def calculateFitnessPopulation(self):
		for p in range(self.num_pop):
			print "[Status]: Testing individo ", p
			self.pop[p].fitness = self.test(self.pop[p])
			print "          Fitness ", self.pop[p].fitness

	def calculateFitnessFilhos(self):
		for p in range(len(self.filhos)):
			print "[Status]: Testing filho ", p
			self.filhos[p].fitness = self.test(self.filhos[p])
			print "          Fitness ", self.filhos[p].fitness


	def torneio(self, size = 2):
		subPop = random.sample(self.pop, size)
		vencedor = subPop.pop(0)
		for ind in subPop:
			if vencedor > ind:
				vencedor = ind
		return vencedor

	def createFilhos(self):
		self.filhos = []
		for x in range(len(self.pop)/2):
			pai_1 = self.torneio(size = 2)
			pai_2 = self.torneio(size = 2)
			(filho1, filho2) = pai_1.cruzamento(pai_2)
			self.filhos.append(filho1)
			self.filhos.append(filho2)

	def selectNextGeneration(self, size_torneio = 2 ):

		todos = self.pop + self.filhos
		todos.sort()

		next_gen = []
		next_gen.append(self.pop[0])

		while len(next_gen) < self.num_pop:
			vencedor_index = min(random.sample(range(len(todos)), size_torneio))
			next_gen.append(todos.pop(vencedor_index))

		self.pop = next_gen

	def updateStatiscs(self):
		population_fitness = [ i.getFitness() for i in self.pop ]
		population_size = [ i.size() for i in self.pop ]
		best_fit = min(population_fitness)
		worst_fit = max(population_fitness)
		average_fit = sum(population_fitness) / float(len(population_fitness))
		
		best_size = min(population_size)
		worst_size = max(population_size)
		average_size = sum(population_size) / float(len(population_size))

		self.statistics.append({'best': best_fit, 'worst': worst_fit, 'average': average_fit,
								'best_size': best_size, 'worst_size': worst_size, 'average_size': average_size })
		print "[Status]: LAST Generation Statistics:"
		print "         ", self.statistics[-1]

		self.showStatistics()
		



	def showStatistics(self):

		best = [ s['best'] for s in self.statistics ]
		worst = [ s['worst'] for s in self.statistics ]
		average = [ s['average'] for s in self.statistics ]
		
		best_size = [ s['best_size'] for s in self.statistics ]
		worst_size = [ s['worst_size'] for s in self.statistics ]
		average_size = [ s['average_size'] for s in self.statistics ]
		
		gen = range(len(best))


		# fitness
		fig = plt.figure(1)
		plt.clf()
		plt.axis([0, len(gen), 0, max(worst)*1.3])

		plt.title('Fitness Evolution')
		plt.xlabel('Generation')
		plt.ylabel('Fitness')

		leg_b, = plt.plot(gen, best,    'r-', label='Best')
		leg_w, = plt.plot(gen, worst,   'b-', label='Worst')
		leg_a, = plt.plot(gen, average, 'g-', label='Average')

		plt.legend(handles=[leg_b, leg_w, leg_a])

		plt.pause(0.0001)

		# tamGenes
		fig = plt.figure(2)
		plt.clf()
		plt.axis([0, len(gen), 0, max(worst_size)*1.3])

		plt.title('Size Evolution')
		plt.xlabel('Generation')
		plt.ylabel('Gene Size')

		leg_b, = plt.plot(gen, best_size,    'r-', label='Best')
		leg_w, = plt.plot(gen, worst_size,   'b-', label='Worst')
		leg_a, = plt.plot(gen, average_size, 'g-', label='Average')

		plt.legend(handles=[leg_b, leg_w, leg_a])

		plt.pause(0.0001)


if __name__ == '__main__':
	try:
		solver = Solver()
		solver.run()
	except rospy.ROSInterruptException:
		pass