#!/usr/bin/env python

import rospy
import os
import sys
import json
import ConfigParser

# Msg structure for the topic communication
from std_msgs.msg import String

config = ConfigParser.ConfigParser()
config.read('config/pddl4j_rospy_config.cfg')
cwd = config.get('options', 'catkinpath')

sys.path.append(cwd + 'devel/lib/python2.7/dist-packages/pddl4j_rospy/')
# Structure for the service communication
from srv import *

# Serialization
import cPickle as pickle

sys.path.append('src/libs/util/')
import AdaptatorPlanJsonPython as adaptator


class Planner:
    #########################################################################
    # PDDL4J-rospy is a ROS implementation for the PDDL4J in python			#
    # library															 	#
    #																		#
    # The ROS node can be used asynchronously by publishing a String 		#
    # on the domain_problem_from_controller_topic topic.					#
    # 																		#
    # It can also be used synchronously by using a simple client/server		#
    # service. In this case, the service to request is 						#
    # serverDomainNameProblem and the answer is going to be a 				#
    # RequestPlannerPlanification											#
    #																		#
    # Use rosrun catkin_planner_PDDL4J catkin_planner_PDDL4J_script.py sync	#
    # to launch the node synchronously										#
    #########################################################################

    # The config object
    config = ConfigParser.ConfigParser()

    PLANNER_PATH = ""

    EXEMPLE_DIRECTORY = ""

    JSON_PATH = ""

    sequentialPlan = "null"

    sync = ""

    def __init__(self, sync="async"):
        print("Initializing the Planner Class....")
        '''
		Initialize the planner by launching the server if sync == "True" / listener if sync == "False"
		:param: string sync: "sync" or "async" 
	    :return: void
		'''
        self.config.read('config/pddl4j_rospy_config.cfg')

        self.PLANNER_PATH = self.config.get('options', 'plannerpath')
        self.EXEMPLE_DIRECTORY = self.config.get('options', 'exempledirectory')
        self.JSON_PATH = self.config.get('options', 'jsonpath')
        #self.JAVA_PLANNER_FACTORY = self.config.get(
        #    'options', 'javaplannerfactory')

        self.sync = sync

        rospy.init_node('PDDL4J_rospy', anonymous=True)

    def setSynchro(self, sync):
        self.sync = sync

    def launch(self):
        try:
            if self.sync == "sync":
                print("Planner launched in synchronous mode...")
                self.serverDomainNameProblem()
            elif self.sync == "async":
                print("Planner launched in asynchronous mode...")
                self.listenerDomainNameProblem()
            else:
                print(
                    "Usage : rosrun pddl4j_rospy"
                    + "PDDL4J_rospy_ros_node.py [sync]")
        except Exception as e:
            if self.sync == "sync":
                print("Planner launched in synchronous mode...")
                self.serverDomainNameProblem()
            elif self.sync == "async":
                print("Planner launched in asynchronous mode...")
                self.listenerDomainNameProblem()
            else:
                print(
                    "Usage : rosrun pddl4j_rospy "
                    + "PDDL4J_rospy_ros_node.py [sync]")

    # 			Communication functions (topic and services)		   #
    ####################################################################
    ###					asynchronous Communication 					###

    def listenerDomainNameProblem(self):
        '''
        listen on the topic domain_problem_from_controller_topic
        It get a String msg structured like [problemDirectory__problemName]
        The callback function is resolvProblemAsTopic which take the data received in parameter
        :param: void
    :return: void
        '''
        rospy.Subscriber("domain_problem_from_controller_topic",
                         String, self.resolvProblemAsTopic)
        print(">> Ready to be requested, waiting a std_msgs/String...")
        print(">> Topic : /domain_problem_from_controller_topic...")
        print(">> Callback : resolvProblemAsTopic...")
        print("############################"
              + "######################################")
        rospy.spin()

    def talkerJsonObject(self, jsonPath, problemResolved, problemDirectory,
                         problemName, jsonPythonObject, operationStatus):
        '''
        create the answer as a String message structured like 
        [jsonPath___problemResolved___problemDirectory___problemName___serializedSequentialPlan___operationStatus] 
        It wait for a Subscriber to listen on the plan_from_pddl4j_topic topic to write the answer
        :param: string jsonPath: The plan as a JSON file path
        :param: boolean problemResolved: True if the problem has been resolved/False otherwise
        :param: string problemDirectory: The problemDirectory received by the listener initialy
        :param: string problemName: The problemName received by the listener initialy
        :param: SequentialPlan jsonPythonObject: The plan as a python object if the problem has been resolved/null otherwise
        :param: string operationStatus: Describing the operation status fileNotFound if the problem or the domain was not found/Ok if the problem is solved  
    :return: void
        '''
        timeToSleepBeforeSending = rospy.Rate(5)
        pub = rospy.Publisher('plan_from_pddl4j_topic', String, queue_size=10)
        if (problemResolved):
            print("The problem has been resolved...\n"
                  + "Sending result to the controller...")
            if not rospy.is_shutdown():
                str_sent = jsonPath + "___True___" + problemDirectory + "___" + problemName + "___" + pickle.dumps(jsonPythonObject, pickle.HIGHEST_PROTOCOL) + "___" + operationStatus
        else:
            print(
                "The problem has not been resolved...\nSending result to the controller...")
            if not rospy.is_shutdown():
                str_sent = jsonPath + "___False___" + problemDirectory + "___" + \
                    problemName + "___" + jsonPythonObject + "___" + operationStatus

        # sleep before checking if someone is subscribed to the topic
        rospy.Rate(10).sleep()
        # waiting for someone to subscribe to the topic before publishing
        while pub.get_num_connections() < 1:
            print("Waiting for someone to connect to the plan_from_pddl4j_topic...")
            rospy.Rate(1).sleep()
        # writing on the topic
        rospy.loginfo(str_sent)
        timeToSleepBeforeSending.sleep()
        pub.publish(str_sent)

        print("Still ready to resolv a problem on the topic...")

    ###					synchronous Communication 					###
    def serverDomainNameProblem(self):
        '''
        Wait to be requested by a RequestPlannerPlanification structure service message
        Call resolvProblemAsService as a callback function
        :param: void 
    :return: void
        '''
        s = rospy.Service('serverDomainNameProblem',
                          RequestPlannerPlanification, self.resolvProblemAsService)
        print(">> Ready to be requested, waiting a RequestPlannerPlanification...")
        print(">> Service : /serverDomainNameProblem...")
        print(">> Callback : resolvProblemAsService...")
        print("##################################################################")
        rospy.spin()

    def formatAndSendAnswerForService(self, jsonPath, problemResolved, problemDirectory, problemName, jsonPythonObject, operationStatus):
        '''
        Format the parameters to be sent in an RequestPlannerPlanification message and send the response to the client waiting
        :param: string jsonPath: The plan as a JSON file path
        :param: boolean problemResolved: True if the problem has been resolved/False otherwise
        :param: string problemDirectory: The problemDirectory received by the listener initialy
        :param: string problemName: The problemName received by the listener initialy
        :param: SequentialPlan jsonPythonObject: The plan as a python object if the problem has been resolved/null otherwise
        :param: string operationStatus: Describing the operation status fileNotFound if the problem or the domain was not found/Ok if the problem is solved  
    :return: string str_sent: [jsonPath___problemResolved___problemDirectory___problemName___serializedSequentialPlan___operationStatus]
        '''
        try:
            print("Generation of the response...")
            if (problemResolved):
                print(
                    "The problem has been resolved...\nSending result to the controller...")
                if not rospy.is_shutdown():
                    str_sent = jsonPath + "___True___" + problemDirectory + "___" + problemName + "___" + pickle.dumps(jsonPythonObject, pickle.HIGHEST_PROTOCOL) + "___" + operationStatus
            else:
                print(
                    "The problem has not been resolved...\nSending result to the controller...")
                if not rospy.is_shutdown():
                    str_sent = jsonPath + "___False___" + problemDirectory + "___" + \
                        problemName + "___" + jsonPythonObject + "___" + operationStatus
            print("Sending the serialized plan to the controller...")
            print(str_sent)
            return str_sent
        except Exception as e:
            print("Service call failed : %s" % e)

    #						Resolution functions					   #
    ####################################################################

    def resolvProblemAsTopic(self, req):
        returnData = self.resolvProblem(req)
        self.talkerJsonObject(returnData[0], returnData[1], returnData[
                              2], returnData[3], returnData[4], returnData[5])

    def resolvProblemAsService(self, req):
        returnData = self.resolvProblem(req)
        return self.formatAndSendAnswerForService(returnData[0], returnData[1], returnData[2], returnData[3], returnData[4], returnData[5])

    # TODO#### use the configuration file or the command line to set the
    # Planner to use
    def resolvProblem(self, req):
        '''
        Try to resolv the problem in req.problemDirectory and req.problemName or in the req.data message by calling core-pddl4j.jar
        The java core is resolving the problem and modifying the json file
        talkerJsonObject is called by giving all the parameters needed to write in the topic
        :param: string req: [problemDirectory__problemName] 
    :return: void
        '''
        print("Exemple directory : " + self.EXEMPLE_DIRECTORY)
        try:
            problemDirectory = req.data.split("__")[0]
            problemName = req.data.split("__")[1]
        except Exception as e:
            problemDirectory = req.problemDirectory
            problemName = req.problemName

        pathProblem = self.EXEMPLE_DIRECTORY + \
            problemDirectory + "/" + problemName + ".pddl"
        pathDomain = self.EXEMPLE_DIRECTORY + problemDirectory + "/domain.pddl"

        operationStatus = "INIT"
        problemResolved = False

        if(os.path.isfile(problemDirectory) and os.path.isfile(problemName)):
            javaCommand = "java -jar " + self.PLANNER_PATH + "src/java/core-pddl4j.jar -o " + \
                problemDirectory + " -f " + problemName + " -json " + self.JSON_PATH
        elif(os.path.isfile(pathDomain) and os.path.isfile(pathProblem)):
            javaCommand = "java -jar " + self.PLANNER_PATH + "src/java/core-pddl4j.jar -o " + \
                pathDomain + " -f " + pathProblem + " -json " + self.JSON_PATH
        else:
            javaCommand = "Error: File not found..."
            operationStatus = "fileNotFound"

        print("javaCommand : " + javaCommand)

        if(operationStatus != "fileNotFound"):
            # Launch the java command
            # use the .jar file giving him the problem, the domain and the path
            # to the json file to create/edit
            os.system(javaCommand)
            try:
                self.sequentialPlan = adaptator.getSequentialPlanFromJson(
                    self.JSON_PATH)
                operationStatus = "Ok"
                problemResolved = True
            except Exception as e:
                print("JSON file could not be parsed\nProblem in the resolution")
                self.sequentialPlan = "False"
                operationStatus = "Error"
        else:
            print("One of the file could not be found...")
            self.sequentialPlan = "False"

        returnData = [self.JSON_PATH, problemResolved, problemDirectory,
                      problemName, self.sequentialPlan, operationStatus]
        return returnData


if __name__ == "__main__":
    try:
        synchrone = sys.argv[0:][1]
    except Exception as ex:
        synchrone = "async"

    planner = Planner()
    planner.setSynchro(synchrone)
    print("synchro : " + synchrone)
    planner.launch()
