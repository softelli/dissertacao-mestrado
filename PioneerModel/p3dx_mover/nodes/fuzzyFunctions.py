from math import *

class mf:
	
	def __init__(self, p):
		'A constructor'
		
		self._p = p[:]
	
		
class trimf(mf):
	'A triangular membership function. Takes three points as constructor argument'

	def evaluate(self,x):
		if x<=self._p[0]:
			return 0
		elif self._p[0] < x <= self._p[1]:
			return (x - self._p[0])/(self._p[1] - self._p[0])
		elif self._p[1] < x <= self._p[2]:
			return 1 - (x - self._p[1])/(self._p[2] - self._p[1])
		else:
			return 0
			
class trapmf(mf):
	'A trapezoidal membership function'
	
	def evaluate(self,x):
		if x<=self._p[0]:
			return 0
		elif self._p[0] < x <= self._p[1]:
			return (x - self._p[0])/(self._p[1] - self._p[0])
		elif self._p[1] < x <= self._p[2]:				
			return 1
		elif self._p[2] < x <= self._p[3]:
			return 1 - (x - self._p[2])/(self._p[3] - self._p[2])
		else:
			return 0
			
class gaussmf(mf):
	'A gaussian membership function'
	
	def evaluate(self,x):
		return exp(-((x-self._p[0])/self._p[1])**2)

					