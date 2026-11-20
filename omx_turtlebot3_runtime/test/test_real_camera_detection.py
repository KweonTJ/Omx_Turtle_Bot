from pathlib import Path
import cv2
import numpy as np
import yaml
import pytest
pytest.importorskip('rclpy')
from omx_turtlebot3_runtime.vision import Detector

ROOT=Path(__file__).resolve().parents[1]
K=np.array([[337.853243,0,178.368987],[0,451.458724,114.459893],[0,0,1.]])
D=np.array([-.389033,.118101,-.000256,-.000826,0.])

def test_real_camera_id0_recovered_while_duplicate_id1_stays_rejected():
    cfg=yaml.safe_load((ROOT/'config/runtime.yaml').read_text())['vision']
    detector=Detector(cfg)
    assert detector.parameters.minMarkerDistanceRate==.01
    for image in sorted((ROOT/'test/fixtures').glob('real_gripper_*.png')):
        poses=detector.detect(cv2.imread(str(image)),K,D)
        raw_corners,raw_ids,_=cv2.aruco.detectMarkers(cv2.imread(str(image),0),detector.dictionary,parameters=detector.parameters)
        raw=[] if raw_ids is None else raw_ids.flatten().tolist()
        expected=[i for i in (0,1) if raw.count(i)==1]
        assert 0 in expected,image.name
        assert sorted(p[0] for p in poses)==expected,image.name
        assert all(p[3]<=cfg['max_reprojection_error_px'] for p in poses)
    assert len(list((ROOT/'test/fixtures').glob('real_gripper_*.png')))==3

def test_distinct_physical_markers_with_same_id_remain_rejected():
    cfg=yaml.safe_load((ROOT/'config/runtime.yaml').read_text())['vision']
    detector=Detector(cfg)
    canvas=np.full((240,320,3),255,np.uint8)
    generator=getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
    marker=generator(detector.dictionary,0,60)
    for x in (30,200):canvas[80:140,x:x+60]=marker[:,:,None]
    assert detector.detect(canvas,K,D)==[]

def test_grouping_fix_does_not_accept_blank_images():
    cfg=yaml.safe_load((ROOT/'config/runtime.yaml').read_text())['vision']
    detector=Detector(cfg)
    for value in (0,127,255):
        assert detector.detect(np.full((240,320,3),value,np.uint8),K,D)==[]
