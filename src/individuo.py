#!/usr/bin/python

import gene
import random

class Individuo(object):
	"""docstring for Individuo"""
	def __init__(self, genes = [], initialSize = 5, rand = False):
		super(Individuo, self).__init__()
		self.genes = genes
		self.initialSize = initialSize
		if rand:
			self.inicializarAleatorio()

	def getGenes(self):
		return self.genes

	def size(self):
		return len(self.genes)

	def getVelocidades(self):
		return [ g.getVelocidadesEnvio() for g in self.genes ]
		
	def inicializarAleatorio(self):
		self.genes = [gene.Gene(rand = True) for x in xrange(self.initialSize)]

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
		g = gene.Gene(rand = True) 
		self.genes.insert(pos, g)

	def _mutAlterar(self):
		pos = random.randint(0, len(self.genes)-1)
		g = gene.Gene(rand = True) 
		self.genes[pos] = g

	def cruzamento(self, parent2):
		parent1 = self
		minLen = min(parent1.size(),parent2.size())

		if minLen <= 1:
			genes1 = parent1.getGenes()
			genes2 = parent2.getGenes()
		else:
			pos = random.randint(1, minLen-1)
			genes1 = parent1.getGenes()[:pos] + parent2.getGenes()[pos:]
			genes2 = parent2.getGenes()[:pos] + parent1.getGenes()[pos:]


		filho1 = Individuo(genes = genes1)
		filho2 = Individuo(genes = genes2)

		return (filho1, filho2)


if __name__ == '__main__':
	ind1 = Individuo(rand = True)
	ind2 = Individuo(rand = True)

	print "ind 1", ind1.getVelocidades()
	print "ind 2", ind2.getVelocidades()
	
	''' como gerar filhos'''
	#filho1, filho2 = ind1.cruzamento(ind2)
	#print "fil 1", filho1.getVelocidades()
	#print "fil 2", filho2.getVelocidades()

	''' como realizar mutacao'''
	#ind1._mutAcrescentar()
	#ind1._mutRemover()
	#ind1._mutAlterar()
	#print "ind 1", ind1.getVelocidades()
	


