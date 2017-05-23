#!/usr/bin/env python

import unittest
import sys

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

	def test_pddl4j_rospy(self):
		print("Tests de pddl4j-rospy en cours de developpement...")
	    """ Creation of Fluents / Exp and Cond Exp in order to test the Action class """
	    
	    '''
	    fluents_positives_exp = Fluents("on", ["a", "b"])
	    fluents_negatives_exp = Fluents("handempty")

	    exp = Exp(fluents_positives_exp, fluents_negatives_exp)

	    fluents_positives_exp_cond_exp_1 = Fluents("on", ["a","b"])
	    fluents_negatives_exp_cond_exp_1 = Fluents("on", ["a", "g"])
	    fluents_positives_exp_cond_exp_2 = Fluents("on", ["e", "f"])
	    fluents_negatives_exp_cond_exp_2 = Fluents("on", ["b", "h"])

	    exp_cond_exp_1 = Exp(fluents_positives_exp_cond_exp_1, fluents_negatives_exp_cond_exp_1)
	    exp_cond_exp_2 = Exp(fluents_positives_exp_cond_exp_2, fluents_negatives_exp_cond_exp_2)

	    cond_exp = CondExp(exp_cond_exp_1, exp_cond_exp_2)

	    fluents_positives_action = Fluents("on", ["g", "h"])
	    fluents_negatives_action = Fluents("on", ["i", "j"])

	    exp_action = Exp(fluents_positives_action, fluents_negatives_action)

	    action = Action("unstack", ["a", "b"], exp_action, cond_exp)

	    fluents_positives_add_1 = Fluents("on", ["n", "v"])
	    fluents_negatives_add_2 = Fluents("on", ["w", "x"])
	    fluents_positives_add_3 = Fluents("on", ["h", "y"])
	    fluents_negatives_add_4 = Fluents("on", ["l", "m"])

	    exp_add_1 = Exp(fluents_positives_add_1, fluents_negatives_add_2)
	    exp_add_2 = Exp(fluents_positives_add_3, fluents_negatives_add_4)

	    cond_exp_add = CondExp(exp_add_1, exp_add_2)

	    action.add_condition_exp(cond_exp_add)

	    second_action = Action("stack", ["b","a"], exp_action, cond_exp)

	    sequential_plan = SequentialPlan([action, second_action])

	    for elem in sequential_plan.actions():
			print elem.affiche()
		'''


unittest.main()