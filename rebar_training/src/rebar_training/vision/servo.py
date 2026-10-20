"""Robot-only Cartesian feedback; accepts targets from the visual task route."""
import torch
from ..expert import CartesianExpert
from ..route import CARRY_JOINTS


class RobotServo:
    # This one method reads robot FK/Jacobians and writes seven commands only.
    # Do not use CartesianExpert.reset/__call__: those are truth-based teachers.
    cartesian_actions = CartesianExpert.cartesian_actions

    def __init__(self, env):
        self.env = env

    def route_actions(self, state, route, goal=None):
        e = self.env
        desired, quaternion, grip = route.targets(state, e.task_spec.insert_goal if goal is None else goal)
        actions = self.cartesian_actions(desired, quaternion, grip)
        measured = e.robot.data.joint_pos.torch[:, e.control_ids[:6]]
        velocity = 1.5*(measured.new_tensor(CARRY_JOINTS)-measured)
        scale = velocity.abs().amax(-1, keepdim=True).clamp_min(e.task_spec.arm_speed)
        actions[route.stage == 4, :6] = (velocity/scale)[route.stage == 4]
        return actions
