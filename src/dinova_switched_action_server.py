#! /usr/bin/env python3
import rospy
import actionlib
import dinova_fabrics_msgs.msg

from dinova_fabrics_wrapper.common_action_server import FabricsActionServer
import rospy
from dinova_fabrics_wrapper.dinova_pose import DinovaFabricsPose
from dinova_fabrics_msgs.msg import FabricsPoseGoal

from switching_functions import time_switching_function
from behaviors import Behavior

class SwitchedFabricsDinovaActionServer(FabricsActionServer):
    def init_specifics(self) -> None:
        self.goal_threshold = rospy.get_param('pose_threshold')
        self._feedback = dinova_fabrics_msgs.msg.FabricsPoseFeedback()
        self._result = dinova_fabrics_msgs.msg.FabricsPoseResult()
        self.nodes = [Behavior("precise", True), Behavior("aggresive", True)]  # list of fabric controllers
        self._action_name = rospy.get_name()
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            dinova_fabrics_msgs.msg.FabricsPoseAction,
            execute_cb=self.execute_cb,
            auto_start = False,
        )
        self._as.start()

        # switching stuff
        self.switching_function = time_switching_function
        self.switch_memory = 0

    def switching_function(self):
        n_controllers = len(self.nodes)  # number of controllers that we can switch in between
        selected_controller = self.switch_memory if self.switch_memory < n_controllers - 1 else 0
        return selected_controller

    def execute_cb(self, goal: FabricsPoseGoal):
        self.node = self.switching_function(self.behaviors, {'switch_memory': self.switch_memory})
        self.node._goal_cb(goal.goal_pose)
        self.goal_threshold = self.node._goal_threshold_cb(goal.goal_threshold)
        self.execute()
        
if __name__ == '__main__':
    server = FabricsDinovaActionServer()
    rospy.spin()