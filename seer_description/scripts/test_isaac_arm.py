#!/usr/bin/env python3
"""Check six-axis joint planning, wrist IK and Cartesian execution in Isaac."""
import argparse
import copy
import json
import time
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetCartesianPath
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.action import ExecuteTrajectory

ARM = ['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--output',default='arm_result.json')
    args=parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('isaac_arm_planning_test')
    node.set_parameters([Parameter('use_sim_time',value=True)])
    state={}; report=[]
    sub=node.create_subscription(JointState,'/joint_states',lambda m:state.update(zip(m.name,m.position)),qos_profile_sensor_data)
    clients={name:node.create_client(cls,name) for name,cls in [('/plan_kinematic_path',GetMotionPlan),('/compute_fk',GetPositionFK),('/compute_ik',GetPositionIK),('/compute_cartesian_path',GetCartesianPath)]}
    execute=ActionClient(node,ExecuteTrajectory,'/execute_trajectory');active=None

    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)

    def wait(future,seconds=20):
        end=time.monotonic()+seconds
        while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if not future.done():raise TimeoutError('MoveIt request timed out')
        return future.result()

    def robot_state(values):
        s=RobotState();s.joint_state.name=list(values);s.joint_state.position=list(values.values());s.is_diff=True
        return s

    def run_trajectory(trajectory,entry):
        nonlocal active
        entry['points']=len(trajectory.joint_trajectory.points)
        if not args.execute:return
        goal=ExecuteTrajectory.Goal();goal.trajectory=trajectory;goal.controller_names=['aubo_arm_controller']
        active=wait(execute.send_goal_async(goal))
        if not active.accepted:raise RuntimeError('Execution rejected')
        result=wait(active.get_result_async(),90);active=None;spin(1)
        last=trajectory.joint_trajectory.points[-1]
        error=max(abs(state[k]-v) for k,v in zip(trajectory.joint_trajectory.joint_names,last.positions))
        entry.update(execution_code=result.result.error_code.val,max_joint_error=error)
        if result.result.error_code.val!=1 or error>.02:raise RuntimeError(str(entry))

    def joint_plan(target,label):
        req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.num_planning_attempts=5;r.allowed_planning_time=5.;r.max_velocity_scaling_factor=.2;r.max_acceleration_scaling_factor=.2;r.start_state=robot_state(state)
        c=Constraints();c.joint_constraints=[JointConstraint(joint_name=k,position=v,tolerance_above=.001,tolerance_below=.001,weight=1.) for k,v in target.items()];r.goal_constraints=[c]
        result=wait(clients['/plan_kinematic_path'].call_async(req)).motion_plan_response
        entry={'test':label,'planning_code':result.error_code.val};report.append(entry)
        if result.error_code.val!=1:raise RuntimeError(str(entry))
        run_trajectory(result.trajectory,entry);print(json.dumps(entry),flush=True)

    def fk(values):
        req=GetPositionFK.Request();req.header.frame_id='base_footprint';req.fk_link_names=['wrist3_Link'];req.robot_state=robot_state(values)
        result=wait(clients['/compute_fk'].call_async(req))
        if result.error_code.val!=1:raise RuntimeError('FK failed')
        return result.pose_stamped[0]

    try:
        for name,client in clients.items():
            if not client.wait_for_service(timeout_sec=15):raise RuntimeError(f'Missing {name}')
        if not execute.wait_for_server(timeout_sec=10):raise RuntimeError('Missing trajectory execution')
        spin(3);initial={k:state[k] for k in ARM};target=dict(initial)
        target['foreArm_joint']+=.15;target['wrist1_joint']-=.10;target['wrist3_joint']+=.12
        joint_plan(target,'six_axis_joint_plan')
        seed=dict(state);seed.update(target)
        goal_values=dict(seed);goal_values['foreArm_joint']+=.03;goal_values['wrist1_joint']-=.03
        endpoint=fk(goal_values)
        req=GetPositionIK.Request();r=req.ik_request;r.group_name='arm';r.ik_link_name='wrist3_Link';r.robot_state=robot_state(seed);r.pose_stamped=endpoint;r.avoid_collisions=True;r.timeout.sec=3
        result=wait(clients['/compute_ik'].call_async(req));report.append({'test':'wrist_pose_ik','code':result.error_code.val})
        if result.error_code.val!=1:raise RuntimeError('Wrist IK failed')
        req=GetCartesianPath.Request();req.header.frame_id='base_footprint';req.group_name='arm';req.link_name='wrist3_Link';req.start_state=robot_state(state if args.execute else seed);req.waypoints=[endpoint.pose];req.max_step=.005;req.avoid_collisions=True
        if hasattr(req,'max_velocity_scaling_factor'):req.max_velocity_scaling_factor=.2;req.max_acceleration_scaling_factor=.2
        result=wait(clients['/compute_cartesian_path'].call_async(req));entry={'test':'cartesian_wrist_path','fraction':result.fraction,'code':result.error_code.val};report.append(entry)
        if result.error_code.val!=1 or result.fraction<.99:raise RuntimeError(str(entry))
        run_trajectory(result.solution,entry);print(json.dumps(entry),flush=True)
        if args.execute:joint_plan(initial,'return_to_start')
        req=GetPositionIK.Request();r=req.ik_request;r.group_name='arm';r.ik_link_name='wrist3_Link';r.robot_state=robot_state(state);r.pose_stamped=copy.deepcopy(endpoint);r.pose_stamped.pose.position.z=10.;r.timeout.sec=1
        result=wait(clients['/compute_ik'].call_async(req))
        report.append({'test':'unreachable_pose_rejected','code':result.error_code.val})
        if result.error_code.val==1:raise RuntimeError('IK accepted an unreachable pose')
    finally:
        if active is not None and active.accepted:wait(active.cancel_goal_async(),5)
        Path(args.output).write_text(json.dumps(report,indent=2));node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
