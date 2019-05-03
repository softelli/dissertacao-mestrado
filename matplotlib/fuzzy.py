from fuzzyFunctions import *
from fuzzySignal import *

class fuzzy:
	'A fuzzy system'
	
	_sigs_in = {}
	
	_rules = []
	
	def __init__(self,name):
		self._name = name
		
	def addInSignal(self,signal):	
		self._sigs_in[signal._name]=signal
		
	def addOutSignal(self,signal):	
		self._sig_out=signal		
		
	def addRule(self,rule):
		self._rules.append(rule)
		
	def evaluate(self,sigs_in):
		ret = {}
			
		for sig in sigs_in.keys():
			ret[sig] = self._sigs_in[sig].evaluate(sigs_in[sig])
	
		vret = {}	
		for key in  self._sig_out._regions.keys():
			
			vret[key] = 0.0
			
		for rule in self._rules:
			aa =  rule.evaluate(ret)
			vret[aa[0]] += aa[1]
		
		return vret
			
class rule:
	'A fuzzy rule'
	
	def __init__(self,ireg,oreg):
		self._ireg = ireg
		self._oreg = oreg		
	
	def evaluate(self,mvals):
		val = 1.0
		
		for rkey in self._ireg.keys():
			mval = mvals[rkey]
			rval = self._ireg[rkey]
			val = val* mval[rval]
		
		return [self._oreg,val]
		