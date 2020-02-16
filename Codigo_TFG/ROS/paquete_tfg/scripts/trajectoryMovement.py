#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

#Codigos de error.
errorDict = {
			1 : 'Invalid position quantity. 6 required.',
			2 : 'Shoulder position value must be between 15 and 165.',
			3 : 'Gripper position value must be between 10 and 63.',
			4 : ' position value must be between 0 and 180.',
			5 : 'No points specified.',
			-1: "Invalid joint name. Name does not exist. The names must be 'base' , 'shoulder', 'elbow', 'wrist_rot', 'wrist_ver' and 'gripper'",
			-2: "Invalid joint name quantity. 6 names required.",
			-3: "Duplicated joint name.",
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


class TMNode:

	def __init__(self):

		self.topicName = 'trajectoryMovementArduino'

		self.pubTMmsg = rospy.Publisher( self.topicName , JointTrajectory , queue_size = 5)
		rospy.Subscriber("trajectoryMovement", JointTrajectory, self.callbackTM)

		self.jointNames = ['base' , 'shoulder' , 'elbow' , 'wrist_rot' , 'wrist_ver' , 'gripper']

		debug("Trajectory Movement node initialized.")
		info( False , "Trajectory Movement node is on.")


	def callbackTM(self , message):

		debug("Message received in 'trajectoryMovement'. Starting verification...")
		debug("Message info: " + str(message))

		joint_names = message.joint_names
		checkResult = self.checkJointNames(joint_names) #Comprobacion de nombres.
		if(checkResult == 0):
			posCheckResult = self.checkPositionsTM(message) #Comprobacion de posiciones.
			if(posCheckResult[0] == 0):

				speedCheckResult = self.checkSpeedTM(message) #Comprobacion de velocidad.
				vCrErrorCode = speedCheckResult[0]
				if(vCrErrorCode == 10):

					info(False , "Correct Message, sending to " + self.topicName)

					#Publica el mensaje en '/trajectoryMovementArduino'.
					self.pubTMmsg.publish(message)
				else:
					info(True , errorDict[vCrErrorCode] + " in point " + str(speedCheckResult[1]) + ".")

			elif(posCheckResult[0] == 4):
				info(True , posCheckResult[1] + errorDict[posCheckResult[0]] + " in point " + str(posCheckResult[2]))

			else:
				info(True , errorDict[posCheckResult[0]] + " in point " + str(posCheckResult[2]))

		else:
			info(True , errorDict[checkResult])

	"""
	---Comprobacion de los nombres.

	Devuelve un codigo de error.

	Codigos de error:
		 0 -> Todo correcto

		-1 -> El nombre no se encuentra entre los valores posibles.
		-2 -> El numero de valores para el campo 'joint_names' es incorrecto
		-3 -> Nombre duplicado.

	"""
	def checkJointNames(self , names):

		debug("Starting joint names checking...")

		checked = []
		errorCode = 0
		if len(names) != len(self.jointNames):
			errorCode = -2
		ind = 0
		while ind < len(names) and errorCode == 0 :
			name = names[ind]
			if(name in self.jointNames):
				if(name in checked):
					errorCode = -3
				else:
					checked.append(name)
			else:
				errorCode = -1
			ind += 1

		debug("Joint names check ended with Error Code -> " + str(errorCode) + ".")

		return errorCode


	"""
	---Comprobacion de las posiciones.

	Devuelve un array compuesto por el codigo de error y el nombre del servomotor y el indice del punto.

	Codigos de error:
		0 -> Todo correcto

		1 -> El array positions no tiene 6 valores.
		2 -> El valor correspondiente al sevomotor shoulder no es correcto
		3 -> El valor correspondiente al servomotor gripper no es correcto
		4 -> El valor correspondiente a otro de los servomotores no es correcto.
		5 -> No hay puntos.

	"""
	def checkPositionsTM(self, message):

		debug("Starting positions checking...")

		joint_names = message.joint_names
		trajectoryPoints = message.points
		errorCode = 0
		name = ''
		if(len(trajectoryPoints) == 0):
			errorCode = 5

		pointInd = 0

		while pointInd < len(trajectoryPoints) and errorCode == 0:
			
			ind = 0
			point = trajectoryPoints[pointInd]

			if(len(point.positions) != 6):
				errorCode = 1
			while ind < len(point.positions) and errorCode == 0:
				value = point.positions[ind]
				name = joint_names[ind]
				if(name == 'shoulder'):
					if(15 > value  or value > 165):
						errorCode = 2
				elif(name == 'gripper'):
					if(10 > value  or value > 73):
						errorCode = 3
				else:
					if(0 > value or value > 180):
						errorCode = 4

				ind += 1

			pointInd += 1

		debug("Positions check ended in point " + str(pointInd) + " with Error Code -> " + str(errorCode) + ".")

		return [errorCode , name , pointInd]

	"""
	---Comprobacion de velocidad.

	Devuelve un array compuesto por un codigo de error y el indice del punto.

	Codigos de error:
		10 -> Todo correcto.

		12 -> Valor incorrecto


	"""	
	def checkSpeedTM(self , message):

		debug("Starting speed checking...")

		errorCode = 10
		pointInd = 0
		trajectory = message.points

		while pointInd < len(trajectory) and errorCode == 10:
			spd = trajectory[pointInd].time_from_start.secs
			
			if(10 > spd or 30 < spd):
				errorCode = 12

			pointInd += 1

		debug("Speed check ended in point " + str(pointInd) + " with Error Code -> " + str(errorCode) + ".")

		return [errorCode , pointInd]


def init():
	rospy.init_node('trajectoryMovement', anonymous=True)
	nodoTM = TMNode()
	rospy.spin()


if __name__ == '__main__':
	init()