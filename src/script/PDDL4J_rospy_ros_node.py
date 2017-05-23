#!/usr/bin/env python
import sys
from PDDL4J_rospy_Planner import Planner

if __name__ == "__main__":
	try:
		synchrone = sys.argv[0:][1]
	except Exception as ex:
		synchrone = "async"
	
	planner = Planner()
	planner.setSynchro(synchrone)
	print("synchro : " + synchrone)
	planner.launch()


