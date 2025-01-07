#! /usr/bin/env python3

import rospy
# from __future__ import print_function
import sys
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import dinova_fabrics_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32

def default_goal(goal):
    #default goal pose:
    goal_position = [3., 0., 1.]
    goal_orientation = [0.0, 0.0, 0.0, 1.0]
    goal.goal_pose.position.x = goal_position[0]
    goal.goal_pose.position.y = goal_position[1]
    goal.goal_pose.position.z = goal_position[2]
    goal.goal_pose.orientation.x = goal_orientation[0]
    goal.goal_pose.orientation.y = goal_orientation[1]
    goal.goal_pose.orientation.z = goal_orientation[2]
    goal.goal_pose.orientation.w = goal_orientation[3]
    
    goal.goal_threshold = 0.01
    return goal

def fabrics_client():
    if len(sys.argv)>1:
        robot_name = sys.argv[1]
    else:
        print("please specify robot name as an argument, e.g.: dingo1, dingo2")
        return
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient(robot_name+'/switched_action_server', dinova_fabrics_msgs.msg.FabricsPoseAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = dinova_fabrics_msgs.msg.FabricsPoseGoal(goal_pose=Pose(), goal_threshold=Float32())
    goal = default_goal(goal=goal)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    max_time = 30.
    status = client.wait_for_result(timeout=rospy.Duration(max_time))
    if status == False:
        client.cancel_goal()
        print("ended after timeout")
    else:
        print("status:", status)
        
        
    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fabrics_client_py')
        result = fabrics_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)