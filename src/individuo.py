#!/usr/bin/python

import gene
import random
import sys

class Individuo(object):
	"""docstring for Individuo"""
	def __init__(self, genes = [], genes_final = [], initialSize = (20,10) , rand = False):
		super(Individuo, self).__init__()
		self.genes = genes
		self.genes_final = genes_final
		self.initialSize = initialSize
		self.fitness = sys.maxint
		self.probability = {'alterar': 0.3, 'acrescentar': 0.3, 'remover': 0.3}

		if rand:
			self.inicializarAleatorio()


	def __gt__(self, other):
		return self.getFitness() > other.getFitness()

	def __lt__(self, other):
		return self.getFitness() < other.getFitness()

	def __le__(self, other):
		return self.getFitness() <= other.getFitness()

	def __ge__(self, other):
		return self.getFitness() >= other.getFitness()

	def getGenes(self):
		return self.genes 

	def getGenesFinal(self):
		return self.genes_final

	def getGenesEnvio(self):
		return self.genes + self.genes_final

	def getFitness(self):
		return self.fitness

	def size(self):
		return (len(self.genes), len(self.genes_final))

	def getVelocidades(self):
		return [ g.getVelocidadesEnvio() for g in self.getGenesEnvio() ]
		
	def inicializarAleatorio(self):
		self.genes = [gene.Gene(rand = True, sentidos = [-1, 1]) for x in xrange(self.initialSize[0])]
		self.genes_final = [gene.Gene(rand = True, sentidos = [-1]) for x in xrange(self.initialSize[1])]

	def mutacao(self):
		if random.random() < self.probability['alterar']:
			self._mutAlterar()
		if random.random() < self.probability['acrescentar']:
			self._mutAcrescentar()
		if random.random() < self.probability['remover']:
			self._mutRemover()

	def _mutRemover(self):
		k = self.size()[0]/float(sum(self.size()))
		if random.random() < k:
			if len(self.genes) <= 5:
				return
			pos = random.randint(0, len(self.genes)-1)
			self.genes.pop(pos)
		else:
			if len(self.genes_final) <= 8:
				return
			pos = random.randint(0, len(self.genes_final)-1)
			self.genes_final.pop(pos)	


	def _mutAcrescentar(self):
		k = self.size()[0]/float(sum(self.size()))
		if random.random() < k:
			pos = random.randint(0, len(self.genes))
			g = gene.Gene(rand = True, sentidos = [-1, 1]) 
			self.genes.insert(pos, g)
		else:
			pos = random.randint(0, len(self.genes_final))
			g = gene.Gene(rand = True, sentidos = [-1]) 
			self.genes_final.insert(pos, g)
		

	def _mutAlterar(self):
		k = self.size()[0]/float(sum(self.size()))
		if random.random() < k:
			pos = random.randint(0, len(self.genes)-1) 
			sentido = [self.genes[pos].getLinear()]
			g = gene.Gene(rand = True, sentidos = [-1,1]) 
			self.genes[pos] = g
		else:
			pos = random.randint(0, len(self.genes_final)-1) 
			sentido = [self.genes_final[pos].getLinear()]
			g = gene.Gene(rand = True, sentidos = [-1]) 
			self.genes_final[pos] = g

	def cruzamento(self, parent2):
		parent1 = self
		minLen = min(parent1.size()[0],parent2.size()[0])
		minLenFim = min(parent1.size()[1],parent2.size()[1])

		k = self.size()[0]/float(sum(self.size()))
		if random.random() < k:
			pos = random.randint(0, minLen-1)
			genes1 = parent1.getGenes()[:pos] + parent2.getGenes()[pos:]
			genes2 = parent2.getGenes()[:pos] + parent1.getGenes()[pos:]
			genes_final1 = parent2.getGenesFinal()
			genes_final2 = parent1.getGenesFinal()
		else:
			pos = random.randint(0, minLenFim-1)
			genes1 = parent1.getGenes()
			genes2 = parent2.getGenes()
			genes_final1 = parent1.getGenesFinal()[:pos] + parent2.getGenesFinal()[pos:]
			genes_final2 = parent2.getGenesFinal()[:pos] + parent1.getGenesFinal()[pos:]
		

		filho1 = Individuo(genes = genes1, genes_final = genes_final1)
		filho2 = Individuo(genes = genes2, genes_final = genes_final2)

		filho1.mutacao()
		filho2.mutacao()

		return (filho1, filho2)




