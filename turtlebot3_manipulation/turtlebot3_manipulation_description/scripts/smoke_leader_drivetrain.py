#!/usr/bin/env python3
"""Isolated ROS test of four display rotors; never publishes hardware commands.

Run after sourcing ROS Humble. Only robot_state_publisher is started, in a
localhost-only domain with private TF and joint-state topics. No RViz/driver.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,default=Path(__file__).resolve().parents[1]/'urdf/leader_turtlebot3.urdf')
    parser.add_argument('--domain',type=int,default=221)
    args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY']='1'
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    topic='/leader_drivetrain_check'
    with tempfile.TemporaryDirectory(prefix='leader_ros_smoke_') as directory:
        os.environ['ROS_LOG_DIR']=directory
        params=Path(directory)/'params.json'
        params.write_text(json.dumps({'/**':{'ros__parameters':{'robot_description':args.urdf.read_text()}}}))
        log=open(Path(directory)/'rsp.log','w+')
        proc=subprocess.Popen([
            '/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher',
            '--ros-args','--params-file',str(params),'-r','__node:=leader_drivetrain_check_rsp',
            '-r',f'/tf:={topic}/tf','-r',f'/tf_static:={topic}/tf_static',
            '-r',f'joint_states:={topic}/joint_states','-r',f'robot_description:={topic}/robot_description',
        ],stdout=log,stderr=subprocess.STDOUT)
        rclpy.init()
        node=rclpy.create_node('leader_drivetrain_test')
        dynamic,static={},{}
        node.create_subscription(TFMessage,topic+'/tf',lambda m:dynamic.update({t.child_frame_id:t for t in m.transforms}),10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(TFMessage,topic+'/tf_static',lambda m:static.update({t.child_frame_id:t for t in m.transforms}),qos)
        pub=node.create_publisher(JointState,topic+'/joint_states',10)
        try:
            for angles in ((0.,0.),(.4,-.6)):
                dynamic.clear()
                deadline=time.monotonic()+10
                done=False
                while time.monotonic()<deadline:
                    if proc.poll() is not None:
                        log.seek(0)
                        raise RuntimeError(log.read())
                    msg=JointState()
                    msg.header.stamp=node.get_clock().now().to_msg()
                    msg.name=['wheel_left_joint','wheel_right_joint','joint1','joint2','joint3','joint4','gripper_left_joint']
                    msg.position=[*angles,0.,-1.15,.6,.55,.01]
                    pub.publish(msg)
                    rclpy.spin_once(node,timeout_sec=.1)
                    done=True
                    for side,angle in zip(('left','right'),angles):
                        for end in ('front','rear'):
                            t=dynamic.get(f'sprocket_{end}_{side}_link')
                            if t is None:
                                done=False
                                continue
                            q=t.transform.rotation
                            done &= abs(q.y-math.sin(angle/2))<1e-7 and abs(q.w-math.cos(angle/2))<1e-7
                    if done and len(static)>=6:
                        break
                assert done, 'Four rotor transforms did not follow the two side states'
                for side,sign in (('left',1),('right',-1)):
                    belt=static[f'track_{side}_link']
                    assert belt.header.frame_id=='base_link'
                    assert abs(belt.transform.translation.y-sign*.0925)<1e-9
                    assert belt.transform.rotation.w==1
                    assert f'track_{side}_link' not in dynamic, 'Belt is rotating with wheel'
                    for end in ('front','rear'):
                        rotor=dynamic[f'sprocket_{end}_{side}_link']
                        assert rotor.header.frame_id==f'motor_{end}_{side}_link'
                        assert all(abs(getattr(rotor.transform.translation,a))<1e-9 for a in 'xyz')
                        assert f'motor_{end}_{side}_link' in static
                        motor=static[f'motor_{end}_{side}_link']
                        assert abs(motor.transform.translation.x-(0. if end=='front' else -.1385))<1e-9
                        assert abs(motor.transform.translation.y-sign*.0925)<1e-9
                        assert abs(motor.transform.translation.z-.023)<1e-9
                    separation=static[f'motor_front_{side}_link'].transform.translation.x-static[f'motor_rear_{side}_link'].transform.translation.x
                    assert abs(separation-2*.03525-.068)<1e-9, 'Facing case gap is not 68 mm'
                for end in ('front','rear'):
                    left=static[f'motor_{end}_left_link'].transform.translation.y
                    right=static[f'motor_{end}_right_link'].transform.translation.y
                    assert abs(left-right-.185)<1e-9, 'Lateral axle separation is not 185 mm'
                    assert abs(left-right-2*.045-.095)<1e-9, 'Lateral case gap is not 95 mm'
                print(f'PASS side angles {angles}: 4 coaxial rotors follow; 4 motor bodies and 2 belts stay fixed')
            for name in ('/joint_states','/tf','/tf_static','/robot_description','/cmd_vel'):
                assert not node.get_publishers_info_by_topic(name), f'Unexpected public publisher: {name}'
            for child,parent,xyz in (
                ('camera_plate_link','base_link',(-.064,0,.11725)),
                ('upper_waffle_plate','base_link',(-.092,0,.18125)),
                ('link1','base_link',(-.128,0,.18175)),
                ('opencr_link','base_link',(-.032,0,.05775)),
                ('jetson_orin_nano_link','camera_plate_link',(-.026,0,.0045)),
            ):
                t=static[child]
                assert t.header.frame_id==parent
                assert all(abs(getattr(t.transform.translation,a)-v)<1e-9 for a,v in zip('xyz',xyz)), child
            print('PASS measured stack: live plate, arm and board TF heights')
            print(f'PASS domain {args.domain}, localhost only, no public robot topics or commands')
        finally:
            node.destroy_node()
            rclpy.shutdown()
            proc.send_signal(signal.SIGINT) if proc.poll() is None else None
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            log.close()


if __name__=='__main__':
    main()
