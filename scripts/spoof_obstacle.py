#!/usr/bin/env python3

import rospy
from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def create_spoofed_obstacle():
    # Create the ObjectArray message
    object_array = ObjectArray()
    object_array.header.stamp = rospy.Time.now()
    object_array.header.frame_id = "map"  # Adjust as needed

    # Create the Object message
    obj = Object()
    obj.id = 1  # Arbitrary ID
    obj.header.stamp = rospy.Time.now()
    # obj.header.frame_id = "map"  # Use a relevant frame

    # Set the position of the obstacle
    obj.pose = Pose()
    obj.pose.position.x = 2.9  # Adjust coordinates as needed
    obj.pose.position.y = -1.5
    obj.pose.position.z = 0.0  # Sphere center height

    # Define the shape as a sphere
    sphere_shape = SolidPrimitive()
    sphere_shape.type = SolidPrimitive.SPHERE
    sphere_shape.dimensions = [0.5]  # Radius = 0.5 meters
    obj.shape = sphere_shape

    # Append to the object array
    object_array.objects.append(obj)

    return object_array

def obstacle_publisher():
    rospy.init_node("spoof_obstacle_publisher", anonymous=True)
    pub = rospy.Publisher('/dingo1/objects', ObjectArray, queue_size=1)

    rate = rospy.Rate(1)  # 1 Hz publishing rate

    while not rospy.is_shutdown():
        obstacle_msg = create_spoofed_obstacle()
        pub.publish(obstacle_msg)
        rospy.loginfo("Published spoofed obstacle")
        rate.sleep()

if __name__ == "__main__":
    try:
        obstacle_publisher()
    except rospy.ROSInterruptException:
        pass
