#! /usr/bin/env python3

import rospy
import subprocess
import sys
import actionlib
import yaml

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import dinova_fabrics_msgs.msg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from switched_control.srv import SetSwitchingConfig
import numpy as np

make_rosbag = True

def default_goal(goal):
    goal.goal_joints.data = rospy.get_param('joint_goal', [2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    goal.goal_threshold = rospy.get_param('goal_threshold', 0.01)
    return goal

def set_switching_config(behaviors, switching_function):
    rospy.wait_for_service('/switched_action_server/set_switching_config')
    try:
        set_config = rospy.ServiceProxy('/switched_action_server/set_switching_config', SetSwitchingConfig)
        response = set_config(behaviors, switching_function)
        if response.success:
            rospy.loginfo(f"Configuration applied: {response.message}")
        else:
            rospy.logerr(f"Failed to apply configuration: {response.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

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
        rospy.set_param('joint_goal', [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # home
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
        goals = [[3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [2.0, 2.0, 0.0, 0.0, -1.5, 0.15, 0.0, 1.0, 0.0]]
        topics_to_record = ["/dinova/omni_states",
                            "/dinova/switched_action_server/switching_signal",
                            "/dingo1/dinova/omni_states_vicon",
                            "/dingo1/switched_action_server/switching_signal",
                            "/dingo2/dinova/omni_states_vicon",
                            "/dingo2/switched_action_server/switching_signal"] # for sim /dinova/omni_states for real "/dingo1/dinova/omni_states_vicon"
        rosbag_file_name = "experiment_data.bag"

        try:
            if make_rosbag:
                # Start recording a rosbag
                print("Starting rosbag recording.")
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
            if make_rosbag:
                # Stop recording the rosbag
                rosbag_process.terminate()
                rosbag_process.wait()
                print("Rosbag recording stopped.")
    elif schedule == "experiment2":
        yaml_file = '/home/leon/dinova_ws/experiments/experiment_settings.yaml'
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        settings = data["settings"]
        topics_to_record = ["/dinova/omni_states",
                            "/dinova/switched_action_server/switching_signal",
                            "/dingo1/dinova/omni_states_vicon",
                            "/dingo1/switched_action_server/switching_signal",
                            "/dingo2/dinova/omni_states_vicon",
                            "/dingo2/switched_action_server/switching_signal",
                            "/dingo1/dingo_velocity_controller/cmd_vel",
                            "/dingo2/dingo_velocity_controller/cmd_vel"]
        
        configuration_initial = settings["start_position"]
        configuration_goal = settings["goal_position"]
        
        # spoof an obstacle at each specified obstacle location and size
        obstacle_data = [j for obst in settings["obstacles"] for j in [str(obst[0]), str(obst[1]), str(obst[2])]]
        spoofed_obst_node = subprocess.Popen(["rosrun", "switched_control", "spoof_obstacle.py", *obstacle_data])

        for experiment in data["experiments"]:
            print(f"\nNow running: {experiment['name']}")
            for run_index in range(settings["number_of_runs"]):
                print(f"Run number {run_index + 1}")
                for i, subexperiment in enumerate(experiment.get("subexperiments", [])):
                    print(f"\tRunning subexperiment {i + 1}/{len(experiment.get('subexperiments', []))}")
                    behavior_name_dict = {"precise_behavior": "p",
                                        "aggressive_behavior": "a",
                                        "precise_aggressive_behaviors": "pa",
                                        "precise_O0": "precise_O0",
                                        "precise_O1": "precise_O1",
                                        "precise_O0O1": "precise_O0O1"}
                    environment_dict = {"sim": "S", "lab": "L"}
                    rosbag_file_name = settings["saving_directory"] + environment_dict[settings["environment"]] + "_" + experiment["name"] +  \
                        "_SubE" + behavior_name_dict[subexperiment["name"]] + "_R" + str(run_index + 1) + ".bag"
                    try:
                        if make_rosbag:
                            # Start recording a rosbag
                            print("\tStarting rosbag recording.")
                            rosbag_process = subprocess.Popen(["rosbag", "record", "-O", rosbag_file_name] + topics_to_record,
                                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                        # configure switching behaviors
                        set_switching_config(subexperiment["behaviors"], subexperiment["switching_function"])
                        
                        # go to goal
                        rospy.set_param('joint_goal', configuration_goal)
                        goal_msg = dinova_fabrics_msgs.msg.FabricsJointGoal(goal_joints=Float64MultiArray(), goal_threshold=Float32())
                        client.send_goal(default_goal(goal=goal_msg))
                        max_time = 30.
                        status = client.wait_for_result(timeout=rospy.Duration(max_time))
                        if status == False:
                            client.cancel_goal()
                            print("ended after timeout")
                        else:
                            print("status:", status)
                    except KeyboardInterrupt:
                        print("Subexperiment interrupted.")
                    
                    # go back to the initial configuration
                    rospy.set_param('joint_goal', configuration_initial)
                    goal_msg = dinova_fabrics_msgs.msg.FabricsJointGoal(goal_joints=Float64MultiArray(), goal_threshold=Float32())
                    client.send_goal(default_goal(goal=goal_msg))
                    max_time = 30.
                    status = client.wait_for_result(timeout=rospy.Duration(max_time))
                    if make_rosbag:
                        # Stop recording the rosbag
                        rosbag_process.terminate()
                        rosbag_process.wait()
                        print("Rosbag recording stopped.")

        spoofed_obst_node.terminate()
    # elif schedule == "draw_vfields":
    #     # drawing the vector field of the policy of the base velocities as a function of the base configuration
    #     start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     goal = [2.0, 2.0, 0.0, 0.0, -1.5, 0.15, 0.0, 1.0, 0.0]
        
    #     res = 10 # (.) samples per meter, (.)^2 samples per m^2
    #     nx = np.linspace(goal[0]-start[0], int((goal[0]-start[0]) * res))
    #     ny = np.linspace(goal[1]-start[1], int((goal[1]-start[1]) * res))
    #     xx, yy = np.meshgrid(nx, ny)
        
    #     for behavior in behaviors:
    #         control_inputs = np.empty() # in this case, demanded reference velocities

    #         for xi in xx:
    #             for yi in yy:
    #                 behavior._q = [xi, yi, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #                 behavior._qdot = [1., 1., 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #                 behavior.set_runtime_arguments()
    #                 action = behavior._planner.compute_action(**behavior._arguments)
    #                 behavior._action = behavior._alpha_action * behavior._action + (1-behavior._alpha_action) * action
    #                 behavior.action_dingo = behavior.rotate_base_action(theta=behavior._q[2], action_vicon=behavior._action)

    #                 if np.linalg.norm(behavior.action_dingo[0:2]) > behavior._dingo_vel_limit:
    #                     behavior.action_dingo[0:2] = behavior.action_dingo[0:2] / np.linalg.norm(behavior.action_dingo[0:2]) * behavior._dingo_vel_limit
                    


            


        
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