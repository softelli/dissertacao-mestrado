from fuzzyFunctions import *

class fuzzySignal:
	'A fuzzy input/ output signal'
	
	_regions = {}
	
	def __init__(self,name):
		'A constructor'
		
		self._name = name
		
	def addRegion(self,name,mf):
		'Adds a new fuzzy region with given name and membership function'
		
		self._regions[name]=mf
		
	def evaluate(self,x):
		regs = self._regions.keys()
		ret = {}
		for key in regs:
			mf = self._regions[key]
			ret[key] = mf.evaluate(x)
			
		return ret 
		
		