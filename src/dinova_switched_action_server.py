#! /usr/bin/env python3
import rospy
import actionlib
import dinova_fabrics_msgs.msg

from dinova_fabrics_wrapper.common_action_server import FabricsActionServer
import rospy
from dinova_fabrics_msgs.msg import FabricsPoseGoal

from behaviors import Behavior
import switching_functions as sfs

class SwitchedFabricsDinovaActionServer(FabricsActionServer):
    def init_specifics(self) -> None:
        self.goal_threshold = rospy.get_param('pose_threshold')
        self._feedback = dinova_fabrics_msgs.msg.FabricsPoseFeedback()
        self._result = dinova_fabrics_msgs.msg.FabricsPoseResult()
        self.nodes = [Behavior("precise", False), Behavior("aggressive", False)]  # list of fabric controllers
        self._action_name = rospy.get_name()
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            dinova_fabrics_msgs.msg.FabricsPoseAction,
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

    def execute_cb(self, goal: FabricsPoseGoal):
        for node in self.nodes:
            node._goal_cb(goal.goal_pose)
        self.goal_threshold = 0.1 #self.node._goal_threshold_cb(goal.goal_threshold)  # TODO: this should instead be defined in the constructer and be written to the node (this only overwrites what is done in the constructor)
        self.execute()  # this will run a while loop (until preempted) that continuously calls run_fabrics()
        
if __name__ == '__main__':
    server = SwitchedFabricsDinovaActionServer()
    rospy.spin()