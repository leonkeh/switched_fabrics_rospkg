#!/usr/bin/env python3

import sys
import rospy
from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def create_object_msg(x, y, r):
    # Create the Object message
    obj = Object()
    obj.id = int(((float(x)+1)*(float(y)+1)*(float(r)+1))**2)  # Arbitrary ID
    obj.header.stamp = rospy.Time.now()
    # obj.header.frame_id = "map"  # Use a relevant frame

    # Set the position of the obstacle
    obj.pose = Pose()
    obj.pose.position.x = float(x)
    obj.pose.position.y = float(y)
    obj.pose.position.z = 0.0  # Sphere center height

    # Define the shape as a sphere
    sphere_shape = SolidPrimitive()
    sphere_shape.type = SolidPrimitive.SPHERE
    sphere_shape.dimensions = [float(r)]  # Radius in meters
    obj.shape = sphere_shape
    return obj

def create_spoofed_obstacles_array(obstacles):
    # Create the ObjectArray message
    object_array = ObjectArray()
    object_array.header.stamp = rospy.Time.now()
    object_array.header.frame_id = "map"  # Adjust as needed

    for obstacle in obstacles:
        object_array.objects.append(obstacle)
    return object_array

def obstacle_publisher():
    rospy.init_node("spoof_obstacle_publisher", anonymous=True)
    n_obstacles = (len(sys.argv) - 1) // 3

    obstacles_msgs = []
    for i in range(n_obstacles):
        obstacles_msgs.append(create_object_msg(sys.argv[1 + 3*i], sys.argv[2 + 3*i], sys.argv[3 + 3*i]))
    obstacle_array_msg = create_spoofed_obstacles_array(obstacles_msgs)
    rospy.loginfo(f"Publishing {n_obstacles} at {sys.argv[1:]}")

    pub = rospy.Publisher('/dingo1/objects', ObjectArray, queue_size=1)

    rate = rospy.Rate(1)  # 1 Hz publishing rate

    while not rospy.is_shutdown():
        pub.publish(obstacle_array_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        obstacle_publisher()
    except rospy.ROSInterruptException:
        pass
