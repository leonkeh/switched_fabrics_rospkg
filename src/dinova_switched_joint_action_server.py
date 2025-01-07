#! /usr/bin/env python3
import rospy
import actionlib
import dinova_fabrics_msgs.msg

from dinova_fabrics_wrapper.common_action_server import FabricsActionServer
import rospy
from dinova_fabrics_wrapper.dinova_joint_space import DinovaFabricsJointSpace
from dinova_fabrics_msgs.msg import FabricsJointGoal

from behaviors import Behavior
import switching_functions as sfs

class SwitchedFabricsDinovaActionServer(FabricsActionServer):
    def init_specifics(self) -> None:
        print("DEBUG HERE --------------------")
        self.goal_threshold = rospy.get_param('joint_space_threshold')
        self._feedback = dinova_fabrics_msgs.msg.FabricsJointFeedback()
        self._result = dinova_fabrics_msgs.msg.FabricsJointResult()
        self.nodes = [Behavior("precise", True), Behavior("aggressive", True)]  # list of fabric controllers
        self._action_name = rospy.get_name()
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            dinova_fabrics_msgs.msg.FabricsJointAction,
            execute_cb=self.execute_cb,
            auto_start = False,
        )
        self._as.start()

        # switching stuff
        self.switching_function = sfs.time_switching_function #sfs.time_switching_function
        self.switch_counter = 0
        self.node = self.nodes[0]  # will be defined dynamically in run_fabrics

    def run_fabrics(self) -> None:
        """this overwrites the parent's run_fabrics to accomendate multi-fabrics use"""
        # select the node (fabric) based on the current value of the switching function
        self.node = self.switching_function(self.nodes, {'switch_counter': self.switch_counter})  
        self.switch_counter += 1

        # execute the selected fabric's policy
        self.node.act()
        self.node.request_vel()
        self.node._rate.sleep()

    def execute_cb(self, goal: FabricsJointGoal):
        for node in self.nodes:
            node.set_goal(list(goal.goal_joints.data))
        self.goal_threshold = 0.1 #self.node._goal_threshold_cb(goal.goal_threshold)  # TODO: this should instead be defined in the constructer and be written to the node (this only overwrites what is done in the constructor)
        self.execute()  # this will run a while loop (until preempted) that continuously calls run_fabrics()
        
if __name__ == '__main__':
    server = SwitchedFabricsDinovaActionServer()
    rospy.spin()