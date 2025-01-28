from dinova_fabrics_wrapper.utils.utils_dinova import compose_pose_goal
from dinova_fabrics_wrapper.dinova_joint_space import DinovaFabricsJointSpace
from geometry_msgs.msg import Pose


class Behavior(DinovaFabricsJointSpace):
    def __init__(self, behavior_type: str, obstacles: bool):
        super().__init__()
        if behavior_type == "aggressive":
            self.goal_weight = 2.4
        elif behavior_type == "precise":
            self.goal_weight = 1.
        else:
            raise ValueError("specified behavior type is not known")
        
        if not obstacles:
            print("No obstacles duh")
            self.with_obstacles = obstacles
            self.nr_obstacles = 0

    def _goal_cb(self, msg: Pose):
        self._goal_composition = compose_pose_goal(msg, self.goal_weight)