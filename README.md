##pddl4j_rospy

###1. Contact

Thibault Schaeller

thibault.schaeller@etu.univ-grenoble-alpes.fr

###2. Description

pddl4j_rospy is the ROS implementation of the PDDL4J library (https://github.com/pellierd/pddl4j)
It allows you to run a ROS node and ask him synchronously or asynchonously to solve a problem based on
PDDL language (Planning Domain Description Language)
pddl4j_rospy comes with its own topic and server interface.

###3. How to use the pddl4j_rospy?

pddl4j_rospy is a ROS package. It needs catkin to run.
Install it before pulling this repository.
It also needs the PDDL4J library, you must download it (https://github.com/pellierd/pddl4j) and build it before
launching the node.

####3.1 How to build pddl4j_rospy?

Once you pulled it in your/catkin/directory/src, go to the root of you catkin directory
> cd ~/you_catkin_directory/
And make install the entire directory.
> catkin_make install

####3.2 How to run pddl4j_rospy?

pddl4j_rospy can be run from his directory
> cd ~/your_catkin_directory/src/pddl4j_rospy

You have to specify if you want to run it synchronously, by default it while initialize a listener on his topic
> rosrun pddl4j_rospy PDDL4J_rospy_Planner.py [sync]

####3.2 How to use pddl4j_rospy?

Before using the package, you need to edit the pddl4j_rospy.cfg.
replace the line "plannerpath" by the absolute path to your ROS package.
You also need to edit the exempledirectory line. It is used if you want to solve existing problems (some are given in the src/problems/ directory)
> nano pddl4j_rospy.cfg

To make pddl4j resolve a problem in a determined domain, you have to write a ROS talker (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) 
You can also simulate a ROS talker by using rostopic
> rostopic pub /domain_problem_from_controller_topic std_msgs/String "problemDirectory__problemName"

to publish on the domain_problem_from_controller_topic topic. The format of the message is ["domainPath__problemPath"] where domainPath and problemPath are absolute path to .pddl domain and problem files. 

If you want to solve an exemple, you can just use ["directoryOfTheProblemName__problemName"] for exemple : ["blocksworld__p01"]

The pddl4j_rospy node will wait a Subscriber on his plan_from_pddl4j_topic topic to write the answer. you will have to write a listener to get the answer (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
In order to simulate a subscribe, you can use rostopic
> rostopic echo /plan_from_pddl4j_topic

The result is a serialized representation of a SequentialPlan (class in src/libs/pddl4j_rospy/). To use it as an object you will
need to import the /src/libs/pddl4j_rospy/ content in your project and deserialize it using cPickle
> import cPickle as pickle

> sequentialPlan = loads(result_returned_by_the_planner_in_string)

A basic utilisation of the SequentialPlan object is available in the PDDL4J_rospy_unit_test.py 
in the test_resolvProblem_display_actions function.

```python
#printing the action name and all the parameters of this action
for action in sequentialPlan.actions():
	print("action : " + action._get_name()),
	for parameter in action._get_parameters():
		print(parameter),
	print("\n"),
```

####3.3 Evolution
Working on library to add a PlannerFactory class which will allow you to configure the planner you want to use at the beginning of the node
or in the configuration file (default HSP planner)
Editing the gradle to get another .jar file which will be used by the ROS node. It will launch the factory class with all the parameters it needs
