from LocalObject import LocalObject
import math
import random
from staticParameters import staticParameter

p = staticParameter



class Circumnavigation:

	def __init__(this):

		#forca
		this.force = 1.0

		#Forca dos coeficientes sobre o Robo
		this.coefficienteForce = 3.0

		#Coeficiente de orientacao
		this.orientationCoef = 1.0

		#Coeficiente de Interferencia
		this.interferenceCoef = 1.0

		#Coeficiente de Proximdiade
		this.proximityCoef = 1.0

		#Coeficiente de aceleracao
		this.accelerationCoef = 1.0

		#Coeficiente de Conversao
		this.conversionCoef = 0.0

		#Coeficiente de Velocidade
		this.velocityCoef = 1.0

		#Robo mais proximo  -  ainda nao utilizado
		this.NestedRobot = ""

		#Distancia obtida
		this.obtainedDistanceToRobot = 0.0;

		#Distancia desejada
		this.desiredDistanceToRobot = 0.0;

		#Distancia Minima entre os Robos
		this.minDistanceBetweenRobots = 0.0

		#Menor Distancia
		this.shortestDistance = 0.0

		#Raio obtido
		this.obtainedRadius = 0.0

		#Raio desejado
		this.desiredRadius = 0.0

		#Angulo Obtido em Relacao ao Beacon
		this.obtainedAngleToBeacon = 0.0

		#Angulo Desejado em Relacao ao Beacon
		this.desiredAngleToBeacon = 90.0

		#Angulo Obtido em Relacao ao Robo
		this.obtainedAngleToRobot = 0.0

		#Angulo Desejado em Relacao ao Robo
		this.desiredAngleToRobot = 0.0

		#Angulo de Espelho em Relacao ao Beacon
		this.mirrorAngleToBeacon = 0.0

		#Angulo Mais proximo
		this.closerAngle = 0.0

		#Coeficiente de Relacao entre Raio Desejado e Obtido
		this.relativeDiffInRadius = 0.0

		#Dimensoes do cone do sensor
		this.sensorRadius = 0.0
		this.sensorAngle = 0.0

		#Direcao - herdada do NetLogo - nao eh possivel fazer essa abordagem aqui???
		this.direction = 0.0

		#Heading - herdada do NetLogo - nao eh possivel fazer essa abordagem aqui??
		this.heading = 0.0

		#Beacon detectado
		this.hasBeacon = false

		#Robo detectado
		this.hasRobot = false


	#atualiza a diferenca relativa entre o raio obtido e o desejado
	def updateRelativeDiffInRadius(this):
		if this.desiredRadius == 0:
			this.relativeDiffInRadius = 0.0
		else:
			this.relativeDiffInRadius = (this.desiredRadius - this.obtainedRadius) / (this.desiredRadius * 1.0)

	#obtem o angulo de espelho em relacao ao beacon
	def getMirrorAngle(this):
		halfSensorAngle = this.sensorAngle / 2.0
		return halfSensorAngle - 2.0 * (halfSensorAngle - 90.0)

	#atualiza o coeficiente de velocidade
	def updateVelocityCoef(this):
		this.velocityCoef = this.proximityCoef + 2.0 * this.accelerationCoef * this.interferenceCoef

	#atualiza o coeficiente de conversao
	def updateConversionCoef(this):
		this.mirrorAngleToBeacon = getMirrorAngle()
		halfSensorAngle = this.sensorAngle / 2.0

		if this.obtainedAngleToBeacon >= 90:
			this.conversionCoef = 1 - ((halfSensorAngle - this.obtainedAngleToBeacon) / (halfSensorAngle - 90.0))
		else:
			if this.obtainedAngleToBeacon >= this.mirrorAngleToBeacon:
				this.conversionCoef = ((90.0 - this.obtainedAngleToBeacon)/(90.0 - this.mirrorAngleToBeacon) * -1.0
			else:
				this.conversionCoef = -1.0

	#atualiza o coeficiente de orientacao - verificar, pois o angulo medio agora eh ZERO
	def updateOrientationCoef(this):
		middleAngle = this.halfSensorAngle
		this.orientationCoef = 1 - (this.absoluteValue(middleAngle - this.obtainedAngleToRobot))/ middleAngle  

	#atualiza a Direcao do Robot -- essa abordagem sera utilizada aqui????
	def updateDirection(this):
		coeffs = this.coefficienteForce * this.getRelativeRadiusDiff() - (this.conversionCoef / (interferenceCoef + 0.1))
		this.direction = this.heading + coeffs
		if(this.direction >= 360):
			this.direction = this.direction - 360

	#atualiza a Heading -- essa abordagem sera utilizada aqui???
	def updateHeading(this):
		this.heading = this.direction

	#atualiza Distancia ate o Robo Mais Proximo
	def updateObtainedDistanceToRobot(this):
		this.obtainedDistanceToRobot = 0.0

	#atualiza Angulo obtido ate ao Robo - a implementar!!!!
	def updateObtainedAngleToRobot(this):
		this.obtainedAngleToRobot = 0.0
		if this.obtainedAngleToRobot < 0:
			this.obtainedAngleToRobot = this.obtainedAngleToRobot + 360

	#atualiza o Coeficiente de Interferencia
	def updateInterferenceCoef(this):
		this.interferenceCoef = (this.absoluteValue(this.obtainedRadius - this.desiredRadius)) / this.desiredRadius

	#atualiza o Coeficiente de Proximidade
	def updateProximityCoef(this):
		this.proximityCoef = (this.obtainedDistanceToRobot - this.minDistanceBetweenRobots) / (this.sensorRadius - this.minDistanceBetweenRobots)

	#atualiza o Coeficiente de Aceleracao
	def updateAccelerationCoef(this):
		#se distancia obtida for igual a desejada
		if this.obtainedDistanceToRobot == this.desiredDistanceToRobot:
			this.accelerationCoef = 1.0

		else:
			#se distancia obtida maior que ZERO
			if this.obtainedDistanceToRobot > 0:
				this.accelerationCoef = (this.obtainedDistanceToRobot - this.desiredDistanceToRobot) / (this.sensorRadius - this.desiredDistanceToRobot)
			#senao, deve resultar sempre negativo
			else:
				this.accelerationCoef = -1.0 * (this.obtainedDistanceToRobot - this.minDistanceBetweenRobots) / (this.desiredDistanceToRobot - this.minDistanceBetweenRobots)	

	#retorna o valor absoluto, ou seja sempre positivo
	def absoluteValue(this, value):
		if absoluteValue >= 0.0:
			return value
		else:
			return -value

	#atualiza os valores para a iteracao atual
	def update(this):

		#atualiza o coeficiente de velocidade
		updateVelocityCoef()


		
		#SE detectou o alvo
		if this.hasBeacon:
			#estado = circunavegacao
			#pintar o sinal de vermelh
			#se o "mostrar link" estiver ativado, mostra link
			#coloque a cor do label em verde

			#atualizar o Angulo Obtido
			this.obtainedAngleToBeacon = 0.0  #todo

			#atualizar o Raio Obtido
			this.obtainedRadius = 0.0 #todo			

			#atualizar o Coeficiente de Interferencia
			updateInterferenceCoef()

			#atualizar a diferenca relativa entre raio desejado e obtido
			relativeDiffInRadius()

			#atualizar o Coeficiente de Conversao
			updateConversionCoef()

			#atualiza a direcao
			updateDirection()

			#atualiza orientacao - (verify if it is need)
			updateHeading()

		#SE nao detectou o alvo
		else:
			#estado = busca
			#altere o rotulo
			#se estava circunavegando, exiba um erro por ter perdido a circunavegacao

			#altere a orientacao
			this.heading = this.heading + random.random() * 2.0

		    #coeficiente de interferencia para 1
		    this.interferenceCoef = 1.0

		    #coenficiente de conversao para 0
		    this.conversionCoef = 0.0

		    #fim se

		#se existir robo no cone
		if this.hasRobot:

			#mostra link com o robo
			#atualiza distancia obtida
			updateObtainedDistanceToRobot()

			#atualiza angulo obtido
			updateObtainedAngleToRobot()

			#atualizar coeficiente de orientacao
			updateOrientationCoef()

			#atualizar coeficiente de proximidade
			updateProximityCoef()

			#atualizar coeficiente de aceleracao
			updateAccelerationCoef()


