from collections import deque
from types import SimpleNamespace
import time
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from omx_turtlebot3_runtime import ros_policy_observer as module
from omx_turtlebot3_runtime.core import PoseCache, RobotSample, Phase


class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,message):self.messages.append(message)


def camera_observer(monkeypatch,selected,detections):
    monkeypatch.setattr(module,'paired_camera_info',lambda image,info,frame:info)
    monkeypatch.setattr(module,'synchronized_joints',lambda *args:np.zeros(4))
    events=[]
    node=SimpleNamespace(selected=selected,history=deque(),cache=PoseCache(),target_base=None,
        cfg={'vision':{'optical_frame':'eef_usb_camera_optical_frame'}},
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)),
        bridge=SimpleNamespace(imgmsg_to_cv2=lambda *a,**k:np.zeros((240,320,3),dtype=np.uint8)),
        detector=SimpleNamespace(detect=lambda *args:detections),
        planner=SimpleNamespace(policy_from_base=np.eye(4),object_from_marker=lambda q,t,r:(t,np.array([0.,0.,0.,1.]))),
        aruco_pose=Publisher(),bbox=Publisher(),front_bbox=Publisher(),close_ready=Publisher(),
        saved=float('inf'),count=0,emit=lambda event,**data:events.append((event,data)))
    image=Image();image.header.stamp.sec=9;image.header.stamp.nanosec=900_000_000
    info=CameraInfo(k=[100.,0.,160.,0.,100.,120.,0.,0.,1.],d=[0.]*5)
    return node,image,info,events


def test_id_one_publishes_only_its_pose_and_goal(monkeypatch):
    zero=np.array([-.03,.01,.25]);one=np.array([.03,.01,.25])
    detections=[(0,zero,np.eye(3),.1),(1,one,np.eye(3),.1)]
    node,image,info,events=camera_observer(monkeypatch,1,detections)
    module.Observer.camera(node,image,info)
    assert not any(event=='VISION_REJECTED' for event,_ in events)
    assert len(node.aruco_pose.messages)==1
    assert node.aruco_pose.messages[0].pose.position.x==one[0]
    np.testing.assert_allclose(node.target_base,one)
    latched=next(data for event,data in events if event=='TARGET_GOAL_LATCHED')
    assert latched['selected_id']==1


def test_unselected_marker_cannot_create_a_grasp_target(monkeypatch):
    node,image,info,events=camera_observer(monkeypatch,1,[(0,np.array([0.,0.,.25]),np.eye(3),.1)])
    module.Observer.camera(node,image,info)
    assert not any(event=='VISION_REJECTED' for event,_ in events)
    assert node.target_base is None
    assert not node.aruco_pose.messages and not node.close_ready.messages


def test_policy_requested_object_slot_changes_to_one():
    now=time.monotonic();sample=RobotSample(now,np.zeros(4),np.zeros(4),.019,0.,np.nan,np.nan,(0.,0.),0)
    node=SimpleNamespace(history=deque([sample]),planner=SimpleNamespace(forward=lambda *a:np.array([0.,0.,.2])),
        goal=None,cfg={'estimation':{'deck_center_xy':[-.052,-.320],'deck_top_z':.07}},
        phase=Phase.REACH,cache=PoseCache(),selected=1,previous=np.zeros(4),planned=now,
        obs=Publisher(),input_ready=Publisher(),target_base=None,emit=lambda *a,**k:None)
    module.Observer.tick(node)
    assert len(node.obs.messages)==1
    assert list(node.obs.messages[0].data[35:37])==[0.,1.]
