#! /usr/bin/env python3

from derived_object_msgs.msg import Object, ObjectArray
from shape_msgs.msg import SolidPrimitive

import rospy
from gazebo_msgs.msg import ModelStates


def obstacle_publisher():
    rospy.init_node('obstacle_perception', anonymous=True)
    pub = rospy.Publisher('/dinova/objects', ObjectArray, queue_size=10)

    def model_state_callback(msg):
        object_array = ObjectArray()
        for i, name in enumerate(msg.name):
            if name not in ["dinova", "ground_plane"]:
                obj = Object()
                obj.pose = msg.pose[i]
                sphere_shape = SolidPrimitive()
                sphere_shape.type = SolidPrimitive.SPHERE
                sphere_shape.dimensions = [0.5]
                obj.shape = sphere_shape
                #obj.orientation = msg.pose[i].orientation
                #obj.shape_params = [0.5]  # Adjust for radius
                object_array.objects.append(obj)
        pub.publish(object_array)

    rospy.Subscriber('/gazebo/model_states', ModelStates, model_state_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        obstacle_publisher()
    except rospy.ROSInterruptException:
        pass
