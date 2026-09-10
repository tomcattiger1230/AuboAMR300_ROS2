#!/usr/bin/env python3
"""Validate imported student targets against the complete current MoveIt robot.

Invalid targets are never executed. Only --execute permits Isaac movement.
"""
import argparse
import copy
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity, GetMotionPlan
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import PoseStamped
from aubo_control_gui.motion_client import ARM, GRIPPER


def orientation_error(a,b):
    norm=lambda q:math.sqrt(sum(v*v for v in q))
    dot=abs(sum(x*y for x,y in zip(a,b))/(norm(a)*norm(b)))
    return 2*math.acos(min(1.0,dot))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--targets',default=str(Path(__file__).parents[1]/'config/student_key_positions.json'))
    parser.add_argument('--output',required=True)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    data=json.loads(Path(args.targets).read_text())
    rclpy.init();node=rclpy.create_node('student_key_position_validation')
    node.set_parameters([Parameter('use_sim_time',value=True)])
    state={};received={};active=None
    def on_state(msg):
        for name,value in zip(msg.name,msg.position):
            if math.isfinite(value):state[name]=value;received[name]=time.monotonic()
    subscription=node.create_subscription(JointState,'/joint_states',on_state,qos_profile_sensor_data)
    clients={name:node.create_client(cls,name) for name,cls in [
        ('compute_fk',GetPositionFK),('compute_ik',GetPositionIK),
        ('check_state_validity',GetStateValidity),('plan_kinematic_path',GetMotionPlan)]}
    execute=ActionClient(node,ExecuteTrajectory,'execute_trajectory')
    report={'frame':data['pose_frame'],'tip':data['tip_link'],'execute_requested':args.execute,
            'scene_limit':'Current MoveIt collision geometry; warehouse geometry not fully imported.',
            'targets':[],'return_to_start':None}
    def save():Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def wait(future,timeout=15):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if not future.done():raise TimeoutError('ROS request timed out')
        return future.result()
    def snapshot():
        spin(.25)
        if not all(n in state and time.monotonic()-received[n]<1.5 for n in ARM+GRIPPER):
            raise RuntimeError('Missing fresh robot feedback')
        return dict(state)
    def rs(values):
        r=RobotState();r.joint_state.name=list(values);r.joint_state.position=list(values.values());r.is_diff=True;return r
    def fk(values):
        req=GetPositionFK.Request();req.header.frame_id=report['frame'];req.fk_link_names=[report['tip']];req.robot_state=rs(values)
        result=wait(clients['compute_fk'].call_async(req))
        if result.error_code.val!=1:raise RuntimeError('FK failed')
        p=result.pose_stamped[0].pose
        return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
    def validity(values):
        req=GetStateValidity.Request();req.robot_state=rs(values);req.group_name='arm'
        result=wait(clients['check_state_validity'].call_async(req))
        return {'valid':result.valid,'contacts':[{'bodies':[c.contact_body_1,c.contact_body_2],
                  'depth_m':c.depth,'position':[c.position.x,c.position.y,c.position.z]} for c in result.contacts]}
    def ik(pose,seeds):
        req=GetPositionIK.Request();r=req.ik_request;r.group_name='arm';r.ik_link_name=report['tip'];r.avoid_collisions=True;r.timeout.sec=1
        r.pose_stamped=PoseStamped();r.pose_stamped.header.frame_id=report['frame'];p=r.pose_stamped.pose
        p.position.x,p.position.y,p.position.z=pose[:3]
        q=pose[3:];norm=math.sqrt(sum(v*v for v in q));q=[v/norm for v in q]
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
        codes=[]
        for seed in seeds:
            r.robot_state=rs(seed);res=wait(clients['compute_ik'].call_async(req));codes.append(res.error_code.val)
            if res.error_code.val==1:
                values=dict(seed);values.update(zip(res.solution.joint_state.name,res.solution.joint_state.position))
                return values,codes
        return None,codes
    def plan(values):
        req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.pipeline_id='ompl';r.allowed_planning_time=3.;r.num_planning_attempts=2
        r.max_velocity_scaling_factor=.15;r.max_acceleration_scaling_factor=.15;r.start_state=rs(snapshot())
        c=Constraints();c.joint_constraints=[JointConstraint(joint_name=n,position=values[n],tolerance_above=.001,tolerance_below=.001,weight=1.) for n in ARM]
        r.goal_constraints=[c]
        return wait(clients['plan_kinematic_path'].call_async(req),15).motion_plan_response
    def run(planned,target):
        nonlocal active
        goal=ExecuteTrajectory.Goal();goal.trajectory=planned.trajectory;goal.controller_names=['aubo_arm_controller']
        active=wait(execute.send_goal_async(goal))
        if not active.accepted:active=None;return {'accepted':False}
        res=wait(active.get_result_async(),120);active=None
        actual=snapshot();desired=fk(target);observed=fk(actual)
        return {'accepted':True,'status':res.status,'code':res.result.error_code.val,
                'max_joint_error_rad':max(abs(target[n]-actual[n]) for n in ARM),
                'position_error_m':math.dist(desired[:3],observed[:3]),
                'orientation_error_rad':orientation_error(desired[3:],observed[3:])}
    initial=None
    try:
        for name,client in clients.items():
            if not client.wait_for_service(timeout_sec=10):raise RuntimeError(f'Missing {name}')
        if args.execute and not execute.wait_for_server(timeout_sec=10):raise RuntimeError('Missing execute action')
        spin(2);initial=snapshot();report['initial_joint_state']=initial
        seeds=[initial]
        for angles in data['joint_targets'].values():
            v=dict(initial);v.update(zip(ARM,map(math.radians,angles)));seeds.append(v)
        entries=[(n,p,False) for n,p in data['pose_targets'].items()]+[(n,p,True) for n,p in data['transit_poses'].items()]
        for name,pose,transit in entries:
            entry={'name':name,'transit':transit,'source_pose':pose};report['targets'].append(entry)
            if not transit:
                target=dict(initial);target.update(zip(ARM,map(math.radians,data['joint_targets'][name])))
                modeled=fk(target)
                entry.update(source_joint_degrees=data['joint_targets'][name],fk_pose=modeled,
                    source_position_error_m=math.dist(pose[:3],modeled[:3]),
                    source_orientation_error_rad=orientation_error(pose[3:],modeled[3:]))
            else:
                target,codes=ik(pose,[snapshot(),*seeds]);entry['ik_codes']=codes
                if target is None:
                    entry['outcome']='ik_failed';save();print(name,entry['outcome'],flush=True);continue
            entry['target_joint_radians']={n:target[n] for n in ARM}
            entry['gripper_states']={}
            for label,value in [('open',0.),('closed',.04)]:
                test=dict(target);test.update({n:value for n in GRIPPER});entry['gripper_states'][label]=validity(test)
            valid=validity(target);entry['current_gripper_validity']=valid
            planned=plan(target);entry['planning_code']=planned.error_code.val
            if not valid['valid']:
                entry['outcome']='blocked_collision';entry['executed']=False
                if planned.error_code.val==1:entry['unexpected_planner_acceptance']=True
            elif planned.error_code.val!=1:
                entry['outcome']='planning_failed';entry['executed']=False
            elif args.execute:
                entry['execution']=run(planned,target)
                ex=entry['execution'];entry['executed']=ex.get('code')==1
                entry['outcome']='passed' if entry['executed'] and ex['max_joint_error_rad']<.02 and ex['position_error_m']<.02 else 'execution_failed'
            else:
                entry['outcome']='planned_only';entry['executed']=False
            save();print(name,entry['outcome'],entry['planning_code'],flush=True)
    finally:
        if active is not None:
            wait(active.cancel_goal_async(),5)
            wait(active.get_result_async(),15)
        if args.execute and initial is not None:
            planned=plan(initial)
            report['return_to_start']={'planning_code':planned.error_code.val}
            if planned.error_code.val==1:report['return_to_start']['execution']=run(planned,initial)
        save();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
