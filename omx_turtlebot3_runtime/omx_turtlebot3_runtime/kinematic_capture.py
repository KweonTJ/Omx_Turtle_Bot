"""Read-only, timestamp-bound ArUco/encoder capture for empty-arm calibration."""
import argparse
from collections import deque
import json
from pathlib import Path
import time
import math
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,JointState
from std_msgs.msg import Bool,String
from message_filters import Subscriber,TimeSynchronizer
from cv_bridge import CvBridge
from .core import RobotSample
from .vision import Detector,synchronized_joints,paired_camera_info


class Capture(Node):
    def __init__(self,args):
        super().__init__('omx_kinematic_capture')
        self.args=args;self.history=deque(maxlen=300);self.stage='WAIT';self.done=False
        self.cfg=yaml.safe_load(args.config.read_text())['vision']
        self.detector=Detector(self.cfg);self.bridge=CvBridge();self.count=0;self.saved=-math.inf
        args.output.mkdir(parents=True,exist_ok=False)
        self.log=(args.output/'samples.jsonl').open('x',buffering=1)
        self.ready=self.create_publisher(Bool,'/omx_calibration/vision_ready',10)
        self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
        self.create_subscription(String,'/omx_calibration/stage',self.on_stage,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.image=Subscriber(self,Image,'/eef_camera/image_raw',qos_profile=qos_profile_sensor_data)
        self.info=Subscriber(self,CameraInfo,'/eef_camera/camera_info',qos_profile=qos_profile_sensor_data)
        self.sync=TimeSynchronizer([self.image,self.info],10);self.sync.registerCallback(self.camera)

    def emit(self,event,**data):
        self.log.write(json.dumps(dict(event=event,monotonic=time.monotonic(),**data),
            default=lambda v:v.tolist() if hasattr(v,'tolist') else str(v),allow_nan=False)+'\n')

    def on_stage(self,msg):
        self.stage=msg.data;self.emit('STAGE',stage=self.stage)
        if self.stage.startswith(('COMPLETE','ABORTED')):self.done=True

    def joints(self,msg):
        try:
            index=[msg.name.index(f'joint{i}') for i in range(1,5)]
            q=np.asarray([msg.position[i] for i in index]);dq=np.asarray([msg.velocity[i] for i in index])
            f=msg.name.index('gripper_left_joint');finger=msg.position[f]
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            age=self.get_clock().now().nanoseconds/1e9-stamp
            if not 0<=age<.2 or not np.isfinite(np.r_[q,dq,finger]).all():return
            self.history.append(RobotSample(time.monotonic()-age,q,dq,finger,0.,math.nan,math.nan,(0,0),0))
        except (ValueError,IndexError):pass

    def camera(self,image,info):
        try:
            info=paired_camera_info(image,info,self.cfg['optical_frame'])
            stamp=image.header.stamp.sec+image.header.stamp.nanosec/1e9
            age=self.get_clock().now().nanoseconds/1e9-stamp
            if not 0<=age<=.3:raise ValueError('stale image')
            now=time.monotonic();q=synchronized_joints(self.history,now-age,.06)
            rgb=self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
            det=self.detector.detect(rgb,np.asarray(info.k).reshape(3,3),np.asarray(info.d))
            self.emit('FRAME',stage=self.stage,ros_stamp=stamp,q=q,ids=[d[0] for d in det],
                detections=[dict(id=i,t=t,r=r,reprojection_error=e) for i,t,r,e in det])
            visible=any(i==self.args.reference_id for i,_,_,_ in det)
            self.ready.publish(Bool(data=visible))
            if self.stage.startswith('HOLD') and now-self.saved>=.12 and visible:
                filename=f'frame_{self.count:05d}.png'
                if not cv2.imwrite(str(self.args.output/filename),rgb):raise OSError('image write failed')
                self.emit('SAMPLE',stage=self.stage,frame=filename,ros_stamp=stamp,q=q,
                    detections=[dict(id=i,t=t,r=r,reprojection_error=e) for i,t,r,e in det],
                    camera_matrix=info.k,distortion=info.d)
                self.count+=1;self.saved=now
        except Exception as exc:
            self.ready.publish(Bool(data=False));self.emit('REJECTED',reason=str(exc),
                last_q=None if not self.history else self.history[-1].q,
                last_dq=None if not self.history else self.history[-1].dq,
                history_age=None if not self.history else time.monotonic()-self.history[-1].stamp)


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--reference-id',type=int,choices=[0,1],default=0)
    args,ros=p.parse_known_args();rclpy.init(args=ros);node=Capture(args)
    try:
        while rclpy.ok() and not node.done:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
