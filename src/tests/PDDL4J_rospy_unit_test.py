#!/usr/bin/env python

import unittest
import sys
import cPickle as pickle

#import the Planner class
sys.path.append('src/script')
from PDDL4J_rospy_Planner import Planner

sys.path.append('src/libs/pddl4j_rospy')
from Exp import Exp
from CondExp import CondExp
from Exp import Exp
from Fluents import Fluents
from Action import Action
from SequentialPlan import SequentialPlan

#Structure for the service communication
from catkin_planner_PDDL4J.srv import *

#adaptator which allow you to display the actions of a Plan python object
sys.path.append('src/libs/pddl4j_rospy/')
import AdaptatorPlanJsonPython as adaptator

class test_req_Planner():
	'''
	Test class used to simulate the request sent to the callback function
	in the asynchronous mode
	'''
	data = ""

	def __init__(self, message):
		self.data = message



class test_Planner(unittest.TestCase):
	'''
	Unit test for the PDDL4J_rospy_Planner class
	'''

	planner = Planner()

	def test_resolvProblem_as_topic(self):
		'''
		Testing the resolvProblemAsTopic function
		this function is waiting a data structure like:
		:param: string data: [problemDirectory__problemName] 
		'''
		req = test_req_Planner("blocksworld__p01")
		self.planner.resolvProblem(req)

	def test_resolvProblem_as_topic_file_not_found(self):
		'''
		Testing the resolvProblemAsTopic function
		this function is waiting a data structure like:
		:param: string data: [problemDirectory__problemName] 
		here the files cannot be found
		'''
		req = test_req_Planner("file__notFound")
		self.planner.resolvProblem(req)

	def test_resolvProblem_as_topic_absolute_path(self):
		'''
		Testing the resolvProblemAsTopic function
		this function is waiting a data structure like:
		:param: string data: [problemDirectory__problemName] 
		here the paths are absolute
		'''
		req = test_req_Planner(self.planner.PLANNER_PATH + "src/problems/barman/domain.pddl__" + self.planner.PLANNER_PATH + "src/problems/barman/mojito.pddl");
		self.planner.resolvProblem(req)

	def test_resolvProblem_display_actions(self):
		'''
		Testing the resolvProblemAsTopic function
		and display the actions of the SequentialPlan generated
		:param: string data: [problemDirectory__problemName] 
		'''
		req = test_req_Planner("blocksworld__p01")
		data = self.planner.resolvProblem(req)
		#data is an array :
		#data = [JSON_PATH, problemResolved, problemDirectory, problemName, sequentialPlan, operationStatus]
		sequentialPlan = data[4]

		#printing the action name and all the parameters of this action
		for action in sequentialPlan.actions():
			print("action : " + action._get_name()),
			for parameter in action._get_parameters():
				print(parameter),
			print("\n"),

unittest.main()