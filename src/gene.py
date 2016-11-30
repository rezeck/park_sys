
import random

class Gene(object):
	"""docstring for Gene"""
	def __init__(self, velocidade = (0,0), rand = False):
		super(Gene, self).__init__()
		if not rand:
			self.setVelocidades(velocidade)
		else: 
			self.setVelocidades(self.velocidadeAleatoria()) 

	def __str__(self):
		return str(self.getVelocidades())


	def getVelocidades(self):
		return (self.linear, self.angular)

	def getVelocidadesEnvio(self):
		return self.getVelocidades()

	def setVelocidades(self, velocidade):
		self.linear = velocidade[0]
		self.angular = velocidade[1]

	def setAngular(self, vel):
		self.angular = vel

	def setLinear(self, vel):
		self.linear = vel

	def getAngular(self):
		return self.angular

	def getLinear(self):
		return self.linear

	def velocidadeAleatoria(self):
		linear = random.sample([-1, 1],1)[0]
		angular = random.sample([-1, 0, 1],1)[0]
		return (linear, angular)

		