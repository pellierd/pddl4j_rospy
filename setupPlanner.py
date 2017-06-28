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

		sub = subprocess.Popen(['locate', '*pddl4j-[3-9].[0-9].[0.9].jar'], stdout=subprocess.PIPE)
		pddl4j_path = sub.communicate()[0].split('\n')[0]


		config.add_section('options')

		print("Planner path : " + cwd + '/')
		config.set('options', 'plannerpath', cwd + '/')

		print("Exemple directory path : " + cwd + '/pddl/problems/')
		config.set('options', 'exempledirectory', cwd + '/pddl/problems/')

		print("Path to the plan as .json file : " + cwd + '/src/jsonFiles/plan.json')
		config.set('options', 'jsonpath', cwd + '/src/jsonFiles/plan.json')

		print("Path to catkin directory : " + catkinpath + '/')
		config.set('options', 'catkinpath', catkinpath + '/')

		print("Path to the pddl4j .jar : " + str(pddl4j_path))
		config.set('options', 'pddl4jjarpath', str(pddl4j_path))

		config.write(configFile)
		configFile.close()

def clonePDDL4JFromGit():
	print(">> Downloading the library from https://github.com/pellierd/pddl4j into the ~/pddl4j/ directory...")
	os.system('git clone https://github.com/pellierd/pddl4j ' + str(os.getenv("HOME")) + '/pddl4j')

def buildPDDL4J():
	print(">> Building the library at ~/pddl4j...")
	current_dir = str(os.getcwd())
	os.chdir(str(os.getenv("HOME")) + "/pddl4j")
	os.system("./gradlew build")
	os.chdir(current_dir)

def makeCatkinEnv():
	print(">> Building the catkin environment...")
	current_dir = str(os.getcwd())

	catkinpatharray = current_dir.split('/')
	catkinpatharray.pop()
	catkinpatharray.pop()
	catkinpath = '/'.join(catkinpatharray)
	
	os.chdir(catkinpath)
	os.system("catkin_make")

	os.chdir(current_dir)


def checkVersionPddl4j():
	print(">> Checking the version of the PDDL4J library...")
	print(">> Updating the database...")
	os.system("sudo -S updatedb")
	sub = subprocess.Popen(['locate', '*pddl4j-[3-9].[0-9].[0.9].jar'], stdout=subprocess.PIPE)
	try:
		version = sub.communicate()[0].split('\n')[0].split("/")[-1]
		if version == "":
			print("The PDDL4J library isn't installed, do you want to install the last version? Y/N")
			answ = str(raw_input(">> "))
			while(answ != "Y" and answ != "y" and answ != "N" and answ != "n"):
				print("\tPlease answer by y or Y or n or N")
				answ = str(raw_input(">> "))
			if(answ == "Y" or answ == "y"):
				clonePDDL4JFromGit()
				buildPDDL4J()
			else:
				print("The PDDL4J_rospy_Planner needs the Java library to work properly\nPlease install it before running the node...")
				exit(0)
		else:
			print("Your current .jar version of the PDDL4J library : " + str(version))
	except Exception as e:
		print("error : " + str(e))
		print(">> ## Error while checking the pddl4j version... ##")

		

def setup():
	checkVersionPddl4j()
	makeCatkinEnv()
	editConfigFile()
	print(">> Installation Complete...")


if __name__ == '__main__':
	setup()
