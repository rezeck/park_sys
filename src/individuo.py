#!/usr/bin/python

import gene


class Individuo(object):
	"""docstring for Individuo"""
	def __init__(self, genes = [], initialSize = 5):
		super(Individuo, self).__init__()
		self.genes = genes
		self.initialSize = initialSize

	def getGenes(self):
		return self.genes

	def getVelocidades(self):
		return [ g.getVelocidadesEnvio() for g in self.genes ]
		
	def inicializarAleatorio(self):
		for x in xrange(self.initialSize):
			g = gene.Gene(random = True) 
			self.genes.append(g)
			#print "->", g.getVelocidades()	

	def mutacao(self):
		if True:
			self.mutAlterar()
		elif True:
			self.mutAcrescentar()
		else:
			self.mutRemover()


	def _mutRemover(self):
		if len(self.genes) <= 1:
			return
		pos = random.randint(0, len(self.genes)-1)
		self.genes.pop(pos)

	def _mutAcrescentar(self):
		pos = random.randint(0, len(self.genes))
		g = gene.Gene(random = True) 
		self.genes.insert(pos, g)

	def _mutAlterar(self):
		pos = random.randint(0, len(self.genes)-1)
		g = gene.Gene(random = True) 
		self.genes[pos] = g

	def cruzamento(self, parent2):
		parent1 = self

		minLen = min(len(parent1),len(parent2))
		pos = random.randint(0, len(self.genes)-1)
		
		#genes1 = parent1.getGenes()[:pos] +
		#genes2 =


		filho1 = Individuo()
		filho2 = Individuo()

		return (filho1, filho2)


if __name__ == '__main__':
	ind = Individuo()
	ind.inicializarAleatorio()
	print ind.getVelocidades()

