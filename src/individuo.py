#!/usr/bin/python

import gene


class Individuo(object):
	"""docstring for Individuo"""
	def __init__(self, genes = [], initialSize = 5):
		super(Individuo, self).__init__()
		self.genes = genes
		self.initialSize = initialSize

	def inicializarAleatorio(self):
		for x in xrange(self.initialSize):
			g = gene.Gene(random = True) 
			self.genes.append(g)
			print "->", g.getVelocidades()	

	def mutacao(self):
		if True:
			self.mutAlterar()
		elif True:
			self.mutAcrescentar()
		else:
			self.mutRemover()


	def mutRemover(self):
		pass

	def mutAcrescentar(self):
		pass

	def mutAlterar(self):
		pass


if __name__ == '__main__':
	ind = Individuo()
	ind.inicializarAleatorio()

