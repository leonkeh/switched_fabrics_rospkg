#! /usr/bin/env python3

import rospy
import rospkg
import subprocess

def make_obstacle():
    package_path = rospkg.RosPack().get_path("switched_control")
    urdf_file_path = package_path + "/src/sphere_obstacle.urdf"
    model_name = "obstacle_" + str(rospy.Time.now().nsecs)
    x, y, z = 1.0, 1.0, 0.  # Position of the obstacle in Gazebo

    # Command to spawn the model using rosrun and passing arguments
    command = [
        "rosrun", "gazebo_ros", "spawn_model",
        "-file", urdf_file_path,
        "-urdf",
        "-model", model_name,
        "-x", str(x),
        "-y", str(y),
        "-z", str(z)
    ]
    try:
        # Start the process
        process = subprocess.Popen(command)
        rospy.loginfo("Spawned a sphere obstacle")
        process.wait()
    except Exception as e:
        rospy.logerr(f"Failed to launch the other node: {e}")

if __name__ == "__main__":
    rospy.init_node("obstacle_spawner")
    make_obstacle()
    rospy.loginfo("Exiting obstacle_spawner node.")
    rospy.signal_shutdown("Task completed.")
