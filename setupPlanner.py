#!/usr/bin/env python
import ConfigParser
import subprocess
import os

def editConfigFile():
	print(">> Configuring the Planner...")
	cwd = str(os.getcwd())
	config = ConfigParser.ConfigParser()
	with open('config/pddl4j_rospy_config.cfg', 'w') as configFile:
		
		catkinpatharray = cwd.split('/')
		catkinpatharray.pop()
		catkinpatharray.pop()
		catkinpath = '/'.join(catkinpatharray)


		config.add_section('options')
		config.set('options', 'plannerpath', cwd + '/')
		config.set('options', 'exempledirectory', cwd + '/pddl/problems/')
		config.set('options', 'jsonpath', cwd + '/src/jsonFiles/plan.json')
		config.set('options', 'catkinpath', catkinpath + '/')
		config.write(configFile)
		configFile.close()

def clonePDDL4JFromGit():
	#print("\tDownloading the library from https://github.com/pellierd/pddl4j into the ~/pddl4j/ directory...")
	#sub = subprocess.Popen(['git', 'clone', 'https://github.com/pellierd/pddl4j', str(os.getcwd()) + '/src/libs/pddl4j/'])
	print(""),

def checkVersionPddl4j():
	print(">> Checking the version of the PDDL4J library...")
	sub = subprocess.Popen(['locate', 'pddl4j-3.5.0.jar'], stdout=subprocess.PIPE)
	try:
		version = sub.communicate()[0].split('\n')[0].split("/")[-1]
		if version == "":
			print("You need to install the pddl4j library from https://github.com/pellierd/pddl4j...")
			exit(0)
		else:
			print("\tYour current .jar version of the PDDL4J library : " + str(version))
	except Exception as e:
		print("You need to install the pddl4j library from https://github.com/pellierd/pddl4j...")
		exit(0)
		'''
		print("\tThe PDDL4J library isn't installed, do you want to install the last version? Y/N")
		answ = str(raw_input("\t>> "))
		while(answ != "Y" and answ != "y" and answ != "N" and answ != "n"):
			print("\tPlease answer by y or Y or n or N")
			answ = str(raw_input("\t>> "))
		if(answ == "Y" or answ == "y"):
			clonePDDL4JFromGit()
		else:
			print("\tThe PDDL4J_rospy_Planner needs the Java library to work properly\nPlease install it before running the node...")
			exit(0)
		'''

def setup():
	editConfigFile()
	checkVersionPddl4j()
	print(">> Installation Complete...")


if __name__ == '__main__':
	setup()
