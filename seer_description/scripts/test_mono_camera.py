#!/usr/bin/env python3
"""Check live MV-CH100-60UM geometry, mono encoding and lack of fake depth."""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--width',type=int,default=1024);parser.add_argument('--height',type=int,default=615);parser.add_argument('--output',default='mono_camera_result.json');args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('mono_camera_check');data={};counts={}
    def receive(msg,key):data[key]=msg;counts[key]=counts.get(key,0)+1
    subscriptions=[node.create_subscription(cls,topic,lambda m,key=key:receive(m,key),qos_profile_sensor_data) for cls,topic,key in [(Image,'/camera/image_raw','image'),(CameraInfo,'/camera/camera_info','info')]]
    start=time.monotonic()
    try:
        while time.monotonic()-start<10:rclpy.spin_once(node,timeout_sec=.1)
        assert 'image' in data and 'info' in data,'Missing live image/CameraInfo'
        image,info=data['image'],data['info']
        assert image.encoding=='mono8',image.encoding
        assert (image.width,image.height)==(args.width,args.height)
        assert (info.width,info.height)==(args.width,args.height)
        assert image.header.frame_id==info.header.frame_id=='camera_optical_frame'
        fx=args.width*12/(4096*.00345);fy=args.height*12/(2460*.00345)
        assert abs(info.k[0]-fx)<.01*fx and abs(info.k[4]-fy)<.01*fy,list(info.k)
        depth_publishers=node.count_publishers('/camera/depth/image_raw')
        assert depth_publishers==0,'Monochrome camera must not publish depth'
        frame=CvBridge().imgmsg_to_cv2(image,'mono8')
        assert float(np.std(frame))>1.,'Image appears blank or occluded'
        output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
        cv2.imwrite(str(output.with_suffix('.jpg')),frame)
        report={'width':image.width,'height':image.height,'encoding':image.encoding,'frame':image.header.frame_id,'k':list(info.k),'d':list(info.d),'frames_in_10_wall_seconds':counts['image'],'depth_publishers':depth_publishers,'pixel_stddev':float(np.std(frame))}
        output.write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
