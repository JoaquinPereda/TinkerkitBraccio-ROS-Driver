#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint



#Codigos de error.
errorDict = {

			1 : 'Invalid position quantity. 6 required.',
			2 : 'Shoulder position value must be between 15 and 165.',
			3 : 'Gripper position value must be between 10 and 63.',
			4 : ' position value must be between 0 and 180.',
			12:	"Time from start secs value must be between 10 and 30."
		}

#Nodos publicadores para info y debug.
pubInfo = rospy.Publisher('info' , String , queue_size = 5)
pubDebug = rospy.Publisher('debug' , String , queue_size = 5)

def debug(message):
	txt = "DEBUG -> " + message
	pubDebug.publish(txt)

def info(error , message):
	if(error):
		txt = "ERROR -> " + message
	else:
		txt = "INFO -> " + message

	debug("------------------------------")

	pubInfo.publish(txt)



class SMNode:

	def __init__(self):
		self.topicName = 'simpleMovementArduino'
		self.pubSMmsg = rospy.Publisher( self.topicName, JointTrajectoryPoint , queue_size = 5)
		
		rospy.Subscriber("simpleMovement", JointTrajectoryPoint , self.callbackSM)

		debug("Simple Movement node initialized.")
		info( False , "Simple Movement node is on.")

	 
	def callbackSM(self , mensaje):

		debug("Message received in 'simpleMovement'. Starting verification...")
		debug("Message info: " + str(mensaje))

		positions = mensaje.positions
		checkResult = self.checkPositionsSM( positions) #Comprobacion de las posiciones.

		if(checkResult[0] == 0):
			speedCheckResult = self.checkSpeedSM(mensaje) #Comprobacion de la velocidad.
			if(speedCheckResult == 10):
				info(False , "Correct Message, sending to " + self.topicName)

				#Publica el mensaje en '/simpleMovementArduino'.
				self.pubSMmsg.publish(mensaje)
			else:
				info(True, errorDict[speedCheckResult])

		elif (checkResult[0] == 4):
			info(True,checkResult[1] + errorDict[checkResult[0]])

		else:
			info(True , errorDict[checkResult[0]])			

	"""
	---Comprobacion de las posiciones.

	Devuelve un array compuesto por el codigo de error y el string 'joint'.

	Codigos de error:
		0 -> Todo correcto

		1 -> El array positions no tiene 6 valores.
		2 -> El valor correspondiente al sevomotor shoulder no es correcto
		3 -> El valor correspondiente al servomotor gripper no es correcto
		4 -> El valor correspondiente a otro de los servomotores no es correcto.


	"""
	def checkPositionsSM(self, positions):
		
		debug("Starting positions checking...")

		if(len(positions) != 6):
			errorCode = 1
		else:
			ind = 0
			errorCode = 0
			while ind < len(positions) and errorCode == 0:
				value = positions[ind]
				if(ind == 1):
					if(15 > value  or value > 165):
						errorCode = 2

				elif(ind == len(positions)-1):
					if(10 > value or value > 73):
						errorCode = 3

				else:
					if(0 > value or value > 180):
						errorCode = 4

				ind += 1

		debug("Positions check ended with Error Code -> " + str(errorCode) + ".")

		return [errorCode, 'Joint']

	"""
	---Comprobacion de velocidad.

	Devuelve un codigo de error.

	Codigos de error:
		10 -> Todo correcto.

		12 -> Valor incorrecto


	"""
	def checkSpeedSM(self, mensaje):

		debug("Starting speed checking...")

		errorCode = 10

		spd = mensaje.time_from_start.secs
		if(10 > spd or 30 < spd):
			errorCode = 12

		debug("Speed check ended with Error Code -> " + str(errorCode) + ".")

		return errorCode


def init():
	rospy.init_node('simpleMovement', anonymous=True)
	nodoSM = SMNode()
	rospy.spin()


if __name__ == '__main__':
	init()