#!/usr/bin/env python3
"""Read-only diagnostic of lifted storage targets; never executes or saves presets."""
import argparse,copy,json,math,time
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from moveit_msgs.srv import GetPositionIK,GetStateValidity,GetMotionPlan
from moveit_msgs.msg import RobotState,Constraints,JointConstraint
from aubo_control_gui.motion_client import ARM,GRIPPER

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--targets',required=True);p.add_argument('--output',required=True);args=p.parse_args()
    data=json.loads(Path(args.targets).read_text());rclpy.init();n=rclpy.create_node('storage_lift_diagnostic');state={}
    sub=n.create_subscription(JointState,'/joint_states',lambda m:state.update(zip(m.name,m.position)),qos_profile_sensor_data)
    ik=n.create_client(GetPositionIK,'compute_ik');valid=n.create_client(GetStateValidity,'check_state_validity');plan=n.create_client(GetMotionPlan,'plan_kinematic_path')
    for c in [ik,valid,plan]:
        if not c.wait_for_service(timeout_sec=10):raise RuntimeError('Missing MoveIt service')
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    if not all(k in state for k in ARM+GRIPPER):raise RuntimeError('Missing full joint state')
    def wait(f):
        rclpy.spin_until_future_complete(n,f,timeout_sec=15)
        if not f.done():raise TimeoutError('MoveIt service')
        return f.result()
    def rs(v):
        s=RobotState();s.joint_state.name=list(v);s.joint_state.position=list(v.values());s.is_diff=True;return s
    report=[]
    try:
        for name,pose in data['pose_targets'].items():
            if not name.startswith('存储'):continue
            for lift in [.10,.12]:
                req=GetPositionIK.Request();r=req.ik_request;r.group_name='arm';r.ik_link_name=data['tip_link'];r.avoid_collisions=True;r.timeout.sec=2
                r.pose_stamped.header.frame_id=data['pose_frame'];p=r.pose_stamped.pose
                p.position.x,p.position.y,p.position.z=pose[:3];p.position.z+=lift
                q=pose[3:];norm=math.sqrt(sum(v*v for v in q));p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=[v/norm for v in q]
                seed=dict(state);seed.update(zip(ARM,map(math.radians,data['joint_targets'][name])));r.robot_state=rs(seed)
                solution=wait(ik.call_async(req));entry={'name':name,'lift_m':lift,'ik_code':solution.error_code.val,'executed':False};report.append(entry)
                if solution.error_code.val!=1:continue
                target=dict(seed);target.update(zip(solution.solution.joint_state.name,solution.solution.joint_state.position));entry['joint_radians']={k:target[k] for k in ARM}
                entry['validity']={}
                for label,value in [('open',0.),('closed',.04)]:
                    v=dict(target);v.update({k:value for k in GRIPPER});req=GetStateValidity.Request();req.robot_state=rs(v);req.group_name='arm';res=wait(valid.call_async(req));entry['validity'][label]={'valid':res.valid,'contacts':[(c.contact_body_1,c.contact_body_2) for c in res.contacts]}
                req=GetMotionPlan.Request();r=req.motion_plan_request;r.group_name='arm';r.start_state=rs(state);r.pipeline_id='ompl';r.allowed_planning_time=3.;r.num_planning_attempts=2;r.max_velocity_scaling_factor=.15;r.max_acceleration_scaling_factor=.15
                c=Constraints();c.joint_constraints=[JointConstraint(joint_name=k,position=target[k],tolerance_above=.001,tolerance_below=.001,weight=1.) for k in ARM];r.goal_constraints=[c]
                res=wait(plan.call_async(req));entry['planning_code']=res.motion_plan_response.error_code.val
                print(name,lift,entry['validity'],entry['planning_code'],flush=True)
    finally:
        Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
