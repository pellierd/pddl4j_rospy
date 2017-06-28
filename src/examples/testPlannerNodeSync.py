#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
import cPickle as pickle

sys.path.append('src/libs/util/')
import AdaptatorPlanJsonPython as adaptator

from pddl4j_rospy.srv import *
#using the service of the Planner node
#it will solve the problem synchronously
def client(problemDirectory, problemName):
	print("Waiting for the service...")
	rospy.wait_for_service('serverDomainNameProblem')
	try:
		print("Connection to the planner server...")
		problemSolver = rospy.ServiceProxy('serverDomainNameProblem', RequestPlannerPlanification)

		print("Connection successfull, generation of the solution...")
		response = problemSolver(problemDirectory, problemName)

		callback(response)

	except Exception as e:
		print("Service call failed : %s"%e)


def callback(req):
	#The message received is formated like this :
	#PathToThePlanAsJsonFile___problemResolved___pathToTheDomainFile/NameOfTheExempleDirectory
	#___pathToTheProblemFile/NameOfTheExempleProblem___SerializedVersionOfThePlanAsAnObject___OperationStatus

	#For exemple if we ask to solve the problem mojito.pddl in the barman directory (from the exemple directory)
	#pddl4j_rospy/src/jsonFiles/plan.json___True___barman___mojito___OBJECT-SERIALIZED___Ok

	jsonPath = req.data.split("___")[0]
	problemResolved = req.data.split("___")[1]
	problemDirectory = req.data.split("___")[2]
	problemName = req.data.split("___")[3]
	serializedJsonPythonObject = req.data.split("___")[4]
	operationStatus = req.data.split("___")[5]

	#If the problem has been resolved
	if problemResolved == "True":
		#we generate the object using cPickles
		print("Generation of the json object from the serialized version...")
		sequentialPlan = pickle.loads(serializedJsonPythonObject)
		print("Object generated...")
		#And we display the actions of the plan
		displayAction(sequentialPlan)
	else:
		#Else we look after the operationStatus to know what went wrong
		if operationStatus == "fileNotFound":
			print("The problem " + problemName + " from the directory " + problemDirectory + " could not been solved\n" 
				+ "Because the files could not been found...")
		else:
			print("Something went wrong and the problem could not been solved")


def displayAction(sequentialPlan):
	#printing the action name and all the parameters
	for action in sequentialPlan.actions():
		print("action : " + action._get_name()),
		for parameter in action._get_parameters():
			print(parameter),
		print("\n"),

if __name__ == '__main__':
	#Initialize a ROS node
	rospy.init_node("testPlanner")
	#Ask to resolv the mojito.pddl problem of the barman/domain.pddl domain
	client("barman", "mojito")