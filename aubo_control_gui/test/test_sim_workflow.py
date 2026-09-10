#!/usr/bin/env python3
"""Explicit opt-in Isaac test through the same client that the GUI uses."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from aubo_control_gui.motion_client import MotionClient, ARM, GRIPPER

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--execute',action='store_true',required=True)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    rclpy.init();node=MotionClient()
    node.set_parameters([Parameter('use_sim_time',value=True),Parameter('enable_motion',value=True)])
    report=[]
    node.on_result=lambda msg,error:print(msg,flush=True)
    def until(predicate,timeout=90):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if not predicate():raise TimeoutError('Workflow wait expired')
    def run(target,group,label):
        node.joint_target(target,.15,.15,group)
        until(lambda:not node.busy)
        result=node.last_result
        if result is None or result.result.error_code.val!=1:raise RuntimeError(f'{label} failed: {result}')
        names=ARM if group=='arm' else GRIPPER
        until(lambda:node.fresh(names))
        error=max(abs(node.state[n]-v) for n,v in zip(names,target))
        report.append({'test':label,'code':result.result.error_code.val,'max_error':error})
        if error>.02:raise RuntimeError(str(report[-1]))
    try:
        until(lambda:node.fresh(ARM+GRIPPER) and node.move.server_is_ready(),30)
        initial=[node.state[n] for n in ARM];gripper=[node.state[n] for n in GRIPPER]
        target=list(initial);target[2]+=.10;target[3]-=.07
        run(target,'arm','gui_joint_plan_execute')
        run(initial,'arm','saved_joint_target_return')
        until(lambda:node.pose is not None and time.monotonic()-node.pose_received<1.5)
        p=node.pose.position
        node.pose_target([p.x,p.y,p.z-.005],.15,.15)
        until(lambda:not node.busy)
        if node.last_result is None or node.last_result.result.error_code.val!=1:raise RuntimeError('Pose planning failed')
        report.append({'test':'gui_pose_target','code':node.last_result.result.error_code.val})
        run(initial,'arm','return_after_pose')
        run([.04,.04],'gripper','gui_gripper_close')
        run([0.,0.],'gripper','gui_gripper_open')
        run(gripper,'gripper','gripper_restore')
        # Cancel a slow arm goal; verify no delayed reissue and stable feedback.
        target=list(initial);target[2]+=.3;target[3]-=.2
        node.joint_target(target,.03,.03)
        until(lambda:node.goal_handle is not None or not node.busy)
        if not node.busy:raise RuntimeError('Goal finished before cancellation test')
        until(lambda:max(abs(node.state[n]-v) for n,v in zip(ARM,initial))>.02 or not node.busy)
        if not node.busy:raise RuntimeError('Motion finished before stop test')
        node.stop();until(lambda:not node.busy)
        if node.last_result is None or node.last_result.status != 5:raise RuntimeError('No terminal cancellation result')
        settled=[node.state[n] for n in ARM]
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        drift=max(abs(node.state[n]-v) for n,v in zip(ARM,settled))
        report.append({'test':'cancel_no_reissue','status':node.last_result.status,'drift':drift})
        if drift>.025:raise RuntimeError('Robot continued moving after cancellation')
        run(initial,'arm','return_after_cancel')
    finally:
        if node.busy:
            node.stop()
            until(lambda:not node.busy,15)
        Path(args.output).write_text(json.dumps(report,indent=2))
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
