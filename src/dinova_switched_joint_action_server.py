#! /usr/bin/env python3
import rospy
import actionlib
import dinova_fabrics_msgs.msg

from dinova_fabrics_wrapper.common_action_server import FabricsActionServer
from dinova_fabrics_msgs.msg import FabricsJointGoal
from std_msgs.msg import Int32

from behaviors import Behavior
import switching_functions as sfs
from switched_control.srv import SetSwitchingConfig, SetSwitchingConfigResponse

class SwitchedFabricsDinovaActionServer(FabricsActionServer):
    def init_specifics(self) -> None:
        self.goal_threshold = rospy.get_param('joint_space_threshold')
        self._feedback = dinova_fabrics_msgs.msg.FabricsJointFeedback()
        self._result = dinova_fabrics_msgs.msg.FabricsJointResult()
        self.nodes = []  # list of fabric controllers
        self.available_behaviors = {"precise": Behavior("precise", True),
                                    "aggressive": Behavior("aggressive", True),
                                    "precise_O0": Behavior("precise", False),
                                    "precise_O1": Behavior("precise", True)}
        self.node = self.available_behaviors["precise"]  # will be defined dynamically in run_fabrics
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
        self.swtch_sgnl_pub = rospy.Publisher('switched_action_server/switching_signal', Int32, queue_size=1) # to publish the switching signal
        self.available_switching_funs = {"no_switching": sfs.no_switching_function,
                                         "time_switching": sfs.time_switching_function,
                                         "state_switching": sfs.pos_switching_function}
        rospy.Service('/switched_action_server/set_switching_config', SetSwitchingConfig, self.set_behaviors_callback)
    
    def set_behaviors_callback(self, request):
        behavior_names = request.behaviors
        switching_function_name = request.switching_function
        self.nodes = []
        try:
            for behavior_name in behavior_names:
                self.nodes.append(self.available_behaviors[behavior_name])
            self.node = self.nodes[0]
            if switching_function_name in self.available_switching_funs.keys():
                self.switching_function = self.available_switching_funs[switching_function_name]
            return SetSwitchingConfigResponse(success=True, message=f"Switched to the behaviors {behavior_names} under the {switching_function_name} function.")
        
        except Exception as e:
            rospy.logerr(f"Failed to apply configuration: {e}")
            return SetSwitchingConfigResponse(success=False, message=str(e))

    def run_fabrics(self) -> None:
        """this overwrites the parent's run_fabrics to accomendate multi-fabrics use"""
        # select the node (fabric) based on the current value of the switching function
        dict_params = {'x': self.node._q[:2],
                       'switch_border': [0.7, -0.5, 0.]}
        print(self.node._q[:2])
        switching_signal, self.node= self.switching_function(self.nodes, dict_params)
        # publish switching signal
        self.swtch_sgnl_pub.publish(int(switching_signal))

        # overwrite parameters - temporary workaround
        rospy.set_param('joint_space_goal_weight', self.node.goal_weight)

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