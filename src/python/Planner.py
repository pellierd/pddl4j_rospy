#!/usr/bin/env python

'''
                   GNU LESSER GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. <http://fsf.org/>
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.


  This version of the GNU Lesser General Public License incorporates
the terms and conditions of version 3 of the GNU General Public
License, supplemented by the additional permissions listed below.

  0. Additional Definitions.

  As used herein, "this License" refers to version 3 of the GNU Lesser
General Public License, and the "GNU GPL" refers to version 3 of the GNU
General Public License.

  "The Library" refers to a covered work governed by this License,
other than an Application or a Combined Work as defined below.

  An "Application" is any work that makes use of an interface provided
by the Library, but which is not otherwise based on the Library.
Defining a subclass of a class defined by the Library is deemed a mode
of using an interface provided by the Library.

  A "Combined Work" is a work produced by combining or linking an
Application with the Library.  The particular version of the Library
with which the Combined Work was made is also called the "Linked
Version".

  The "Minimal Corresponding Source" for a Combined Work means the
Corresponding Source for the Combined Work, excluding any source code
for portions of the Combined Work that, considered in isolation, are
based on the Application, and not on the Linked Version.

  The "Corresponding Application Code" for a Combined Work means the
object code and/or source code for the Application, including any data
and utility programs needed for reproducing the Combined Work from the
Application, but excluding the System Libraries of the Combined Work.

  1. Exception to Section 3 of the GNU GPL.

  You may convey a covered work under sections 3 and 4 of this License
without being bound by section 3 of the GNU GPL.

  2. Conveying Modified Versions.

  If you modify a copy of the Library, and, in your modifications, a
facility refers to a function or data to be supplied by an Application
that uses the facility (other than as an argument passed when the
facility is invoked), then you may convey a copy of the modified
version:

   a) under this License, provided that you make a good faith effort to
   ensure that, in the event an Application does not supply the
   function or data, the facility still operates, and performs
   whatever part of its purpose remains meaningful, or

   b) under the GNU GPL, with none of the additional permissions of
   this License applicable to that copy.

  3. Object Code Incorporating Material from Library Header Files.

  The object code form of an Application may incorporate material from
a header file that is part of the Library.  You may convey such object
code under terms of your choice, provided that, if the incorporated
material is not limited to numerical parameters, data structure
layouts and accessors, or small macros, inline functions and templates
(ten or fewer lines in length), you do both of the following:

   a) Give prominent notice with each copy of the object code that the
   Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the object code with a copy of the GNU GPL and this license
   document.

  4. Combined Works.

  You may convey a Combined Work under terms of your choice that,
taken together, effectively do not restrict modification of the
portions of the Library contained in the Combined Work and reverse
engineering for debugging such modifications, if you also do each of
the following:

   a) Give prominent notice with each copy of the Combined Work that
   the Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the Combined Work with a copy of the GNU GPL and this license
   document.

   c) For a Combined Work that displays copyright notices during
   execution, include the copyright notice for the Library among
   these notices, as well as a reference directing the user to the
   copies of the GNU GPL and this license document.

   d) Do one of the following:

       0) Convey the Minimal Corresponding Source under the terms of this
       License, and the Corresponding Application Code in a form
       suitable for, and under terms that permit, the user to
       recombine or relink the Application with a modified version of
       the Linked Version to produce a modified Combined Work, in the
       manner specified by section 6 of the GNU GPL for conveying
       Corresponding Source.

       1) Use a suitable shared library mechanism for linking with the
       Library.  A suitable mechanism is one that (a) uses at run time
       a copy of the Library already present on the user's computer
       system, and (b) will operate properly with a modified version
       of the Library that is interface-compatible with the Linked
       Version.

   e) Provide Installation Information, but only if you would otherwise
   be required to provide such information under section 6 of the
   GNU GPL, and only to the extent that such information is
   necessary to install and execute a modified version of the
   Combined Work produced by recombining or relinking the
   Application with a modified version of the Linked Version. (If
   you use option 4d0, the Installation Information must accompany
   the Minimal Corresponding Source and Corresponding Application
   Code. If you use option 4d1, you must provide the Installation
   Information in the manner specified by section 6 of the GNU GPL
   for conveying Corresponding Source.)

  5. Combined Libraries.

  You may place library facilities that are a work based on the
Library side by side in a single library together with other library
facilities that are not Applications and are not covered by this
License, and convey such a combined library under terms of your
choice, if you do both of the following:

   a) Accompany the combined library with a copy of the same work based
   on the Library, uncombined with any other library facilities,
   conveyed under the terms of this License.

   b) Give prominent notice with the combined library that part of it
   is a work based on the Library, and explaining where to find the
   accompanying uncombined form of the same work.

  6. Revised Versions of the GNU Lesser General Public License.

  The Free Software Foundation may publish revised and/or new versions
of the GNU Lesser General Public License from time to time. Such new
versions will be similar in spirit to the present version, but may
differ in detail to address new problems or concerns.

  Each version is given a distinguishing version number. If the
Library as you received it specifies that a certain numbered version
of the GNU Lesser General Public License "or any later version"
applies to it, you have the option of following the terms and
conditions either of that published version or of any later version
published by the Free Software Foundation. If the Library as you
received it does not specify a version number of the GNU Lesser
General Public License, you may choose any version of the GNU Lesser
General Public License ever published by the Free Software Foundation.

  If the Library as you received it specifies that a proxy can decide
whether future versions of the GNU Lesser General Public License shall
apply, that proxy's public statement of acceptance of any version is
permanent authorization for you to choose that version for the
Library.
'''

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

import AdaptatorPlanJsonPython as adaptator


class Planner:
    #########################################################################
    # PDDL4J-rospy is a ROS implementation for the PDDL4J in python			
    # library															 	
    #																		
    # The ROS node can be used asynchronously by publishing a String 		
    # on the domain_problem_from_controller_topic topic.					
    # 																		
    # It can also be used synchronously by using a simple client/server		
    # service. In this case, the service to request is 						
    # serverDomainNameProblem and the answer is going to be a 				
    # RequestPlannerPlanification											
    #																		
    # Use rosrun catkin_planner_PDDL4J catkin_planner_PDDL4J_script.py sync	
    # to launch the node synchronously										
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
        self.PDDL4J_JAR_PATH = self.config.get(
            'options', 'pddl4jjarpath')

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

    # 			Communication functions (topic and services)		   
    ####################################################################
    ###			Asynchronous Communication 					

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

    ###					Synchronous Communication 					
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

    #						Resolution functions					   
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
            javaCommand = "java -jar " + self.PDDL4J_JAR_PATH + " -o " + \
                problemDirectory + " -f " + problemName + " -json " + self.JSON_PATH
        elif(os.path.isfile(pathDomain) and os.path.isfile(pathProblem)):
            javaCommand = "java -jar " + self.PDDL4J_JAR_PATH + " -o " + \
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
