#! /usr/bin/env python3

import rospy
import subprocess
import sys
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import dinova_fabrics_msgs.msg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32

def default_goal(goal):
    goal.goal_joints.data = rospy.get_param('joint_goal', [2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    goal.goal_threshold = rospy.get_param('goal_threshold', 0.01)
    return goal

def fabrics_client(schedule):
    if len(sys.argv)>1:
        robot_name = sys.argv[1]
    else:
        print("please specify robot name as an argument, e.g.: dingo1, dingo2, dinova (in the simulator)")
        return
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient(robot_name+'/switched_action_server', dinova_fabrics_msgs.msg.FabricsJointAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = dinova_fabrics_msgs.msg.FabricsJointGoal(goal_joints=Float64MultiArray(), goal_threshold=Float32())
    
    if schedule == "single_goal":
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

    elif schedule == "experiment":
        # run an experiment and record the data in a rosbag
        goals = [[1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        topics_to_record = ["/dinova/omni_states"]
        rosbag_file_name = "experiment_data.bag"

        try:
            # Start recording a rosbag
            print("Starting rosbag recording...")
            rosbag_process = subprocess.Popen(
                ["rosbag", "record", "-O", rosbag_file_name] + topics_to_record,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            for config_goal in goals:
                rospy.set_param('joint_goal', config_goal)
                goal = default_goal(goal=goal)
                client.send_goal(goal)
                max_time = 30.
                status = client.wait_for_result(timeout=rospy.Duration(max_time))
                if status == False:
                    client.cancel_goal()
                    print("ended after timeout")
                else:
                    print("status:", status)
        except KeyboardInterrupt:
            print("Experiment interrupted.")
        finally:
            # Stop recording the rosbag
            print("Stopping rosbag recording...")
            rosbag_process.terminate()
            rosbag_process.wait()
            print("Rosbag recording stopped.")

        
    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fabrics_client_py')
        schedule = sys.argv[2] if len(sys.argv) > 2 else "single_goal"
        result = fabrics_client(schedule)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)