from dinova_fabrics_wrapper.dinova_pose import DinovaFabricsPose
from dinova_fabrics_wrapper.utils.utils_dinova import compose_pose_goal
from geometry_msgs.msg import Pose


class Behavior(DinovaFabricsPose):
    def __init__(self, behavior_type: str, obstacles: bool):
        super().__init__()
        if behavior_type == "aggressive":
            self.goal_weight = 20.
        elif behavior_type == "precise":
            self.goal_weight = 20.
        else:
            raise ValueError("specified behavior type is not known")
        
        if obstacles:
            pass

    def _goal_cb(self, msg: Pose):
        self._goal_composition = compose_pose_goal(msg, self.goal_weight)