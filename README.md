##catkin_planner_PDDL4J

###1. Contact

###2. Description
catkin_planner_PDDL4J is the ROS implementation of the PDDL4J library (https://github.com/pellierd/pddl4j)
It allow you to run a ROS node and ask him synchronously or asynchonously to solve a problem based on
PDDL language (Planning Domain Description Language)
It come with its own topic and server interface.

###3. How to use the catkin_planner_PDDL4J?
catkin_planner_PDDL4J is a ROS package. It need catkin to run.
Install it before pulling this package.

####3.1 How to build catkin_planner_PDDL4J?
Once you pulled it in your/catkin/directory/src, go to the root of you catkin directory
>> cd ~/you_catkin_directory/
And make install the entire directory.
>> catkin_make install

####3.2 How to run catkin_planner_PDDL4J?
catkin_planner_PDDL4J can be run from his directory
>> cd ~/your_catkin_directory/src/catkin_planner_PDDL4J

You have to specify if you want to run it synchronously, by default it while initialize a listener on his topic
>> rosrun catkin_planner_PDDL4J catkin_planner_PDDL4J_script.py sync

####3.2 How to use catkin_planner_PDDL4J?
Before using the package, you need to edit the catkin_planner_PDDL4J.cfg.
remplace the line "plannerpath" by the absolute path to your ROS package.
>> nano catkin_planner_PDDL4J.cfg

To make him resolv a problem in a determined domain, you have to write a ROS talker (http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29) 
to publish on the domain_problem_from_controller_topic topic. The format of the message is ["problemDirectory problemName"]

You can also simulate a ROS talker by using rostopic
>> rostopic pub /domain_problem_from_controller_topic std_msgs/String "problemDirectory problemName"

The catkin_planner_PDDL4J node will wait a Subscriber on his plan_from_pddl4j_topic topic to write the answer.
In order to simulate a subscribe, you can use rostopic
>> rostopic echo /plan_from_pddl4j_topic

problemDirectory can be the name of a directory in src/problems/ (Ex: barman) or an absolute path to a domain.pddl file
problemName can be the name of a .pddl file in src/problems/problemDirectory/ (Ex: mojito) or an absolute path to a pb.pddl file 

The result is a serialized representation of a SequentialPlan (class in src/libs/pddl4j_rospy/). To use it as an object you will
need to import the /src/libs/pddl4j_rospy/ content in your project and deserialize it using cPickle
>> import cPickle as pickle

>> sequentialPlan = loads(result_returned_by_the_planner_in_string)