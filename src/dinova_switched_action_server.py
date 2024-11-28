#! /usr/bin/env python3
import rospy
import actionlib
import dinova_fabrics_msgs.msg

from dinova_fabrics_wrapper.common_action_server import FabricsActionServer
import rospy
from dinova_fabrics_wrapper.dinova_pose import DinovaFabricsPose
from dinova_fabrics_msgs.msg import FabricsPoseGoal

class FabricsDinovaActionServer(FabricsActionServer):

    def init_specifics(self) -> None:
        self.goal_threshold = rospy.get_param('pose_threshold')
        self._feedback = dinova_fabrics_msgs.msg.FabricsPoseFeedback()
        self._result = dinova_fabrics_msgs.msg.FabricsPoseResult()
        self.node = DinovaFabricsPose()
        self._action_name = rospy.get_name()
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            dinova_fabrics_msgs.msg.FabricsPoseAction,
            execute_cb=self.execute_cb,
            auto_start = False,
        )
        self._as.start()

    def execute_cb(self, goal: FabricsPoseGoal):
        self.node._goal_cb(goal.goal_pose)
        self.goal_threshold = self.node._goal_threshold_cb(goal.goal_threshold)
        self.execute()
        
if __name__ == '__main__':
    server = FabricsDinovaActionServer()
    rospy.spin()
