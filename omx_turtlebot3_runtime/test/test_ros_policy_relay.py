"""Real ROS/C++ policy path on an isolated domain and a non-actuating topic."""
import os
from pathlib import Path
import subprocess
import time
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray,Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_prefix
from omx_turtlebot3_runtime.cpp_backend import NativeActor

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('max_delta',[.17,.25])
def test_cpp_relay_infers_mirrors_once_and_preserves_direct_handoff(tmp_path,max_delta):
    model=ROOT.parent/'omx_turtlebot3_policy/models/omx_turtlebot3_v6'
    executable=Path(get_package_prefix('omx_turtlebot3_control_cpp'))/'lib/omx_turtlebot3_control_cpp/ros_policy_relay_cpp'
    context=Context();rclpy.init(context=context,domain_id=227)
    node=Node('relay_protocol_test',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    q=np.array([.1,.1,.1,-.3]);obs=np.zeros(69,dtype=np.float32);obs[:4]=q;obs[17]=1
    actor=NativeActor(model,ROOT/'config/runtime.yaml',ROOT/'assets/robot_real.urdf')
    action=actor(obs)
    extension_action=action.copy()
    joints=node.create_publisher(JointState,'/joint_states',10)
    observations=node.create_publisher(Float32MultiArray,'/omx_v6/observation',10)
    raw=node.create_publisher(JointTrajectory,'/arm_controller/joint_trajectory_raw',10)
    direct=node.create_publisher(JointTrajectory,'/omx_v6/nominal_joint_trajectory',10)
    outputs=[]
    sub=node.create_subscription(JointTrajectory,'/isolated_relay_command',lambda m:outputs.append(m),10)
    env=dict(os.environ,ROS_DOMAIN_ID='227',ROS_LOG_DIR=str(tmp_path/'ros'))
    with (tmp_path/'relay.log').open('w') as log:
        process=subprocess.Popen([str(executable),'--ros-args','-p','model_dir:='+str(model),
            '-p','execute:=true','-p','max_raw_joint3_delta_rad:='+str(max_delta),
            '-p','output_topic:=/isolated_relay_command'],env=env,stdout=log,stderr=subprocess.STDOUT)
        def pump(seconds,command=None,publisher=None,send_obs=True):
            end=time.monotonic()+seconds;sent=False
            while time.monotonic()<end:
                assert process.poll() is None,(tmp_path/'relay.log').read_text()
                joints.publish(JointState(name=['joint1','joint2','joint3','joint4'],position=q.tolist()))
                if send_obs:observations.publish(Float32MultiArray(data=obs.tolist()))
                executor.spin_once(timeout_sec=.02)
                if command is not None and not sent and publisher.get_subscription_count()>0:
                    publisher.publish(command);sent=True
            return sent
        def trajectory(value):
            p=JointTrajectoryPoint(positions=[.1,.1,value,-.3]);p.time_from_start.sec=1
            return JointTrajectory(joint_names=['joint1','joint2','joint3','joint4'],points=[p])
        try:
            pump(.8,trajectory(.15),raw,send_obs=False)
            assert not outputs
            pump(.7)
            assert pump(.5,trajectory(.15),raw)
            assert outputs,(tmp_path/'relay.log').read_text()
            np.testing.assert_allclose(outputs[-1].points[0].positions,np.array([.1,.1,.05,-.3])+.00144*extension_action,atol=1e-8)
            q[2]=.12;pump(.1)
            before=len(outputs)
            pump(.4,trajectory(.15),raw)
            assert len(outputs)==before, 'Repeated segment must not restart the controller spline'
            np.testing.assert_allclose(outputs[-1].points[0].positions,np.array([.1,.1,.05,-.3])+.00144*extension_action,atol=1e-8)
            pump(.25)
            pump(.25,trajectory(.15),raw)
            assert len(outputs)>before
            np.testing.assert_allclose(outputs[-1].points[0].positions,np.array([.1,.1,.05,-.3])+.00144*extension_action,atol=1e-8)
            pump(.4,trajectory(.2),direct)
            np.testing.assert_allclose(outputs[-1].points[0].positions,np.array([.1,.1,.2,-.3])+.00144*action,atol=1e-8)
            pump(.4,trajectory(.8),raw)
            np.testing.assert_allclose(outputs[-1].points[0].positions,
                np.array([.1,.1,.12-max_delta,-.3])+.00144*extension_action,atol=1e-8)
            protected_action=action.copy();protected_action[1:]=0.
            for tag,value in [('mp_control_retract_z_hold',.22),('mp_control_pitch_hold',.27)]:
                protected=trajectory(value);protected.header.frame_id=tag
                pump(.4,protected,direct)
                np.testing.assert_allclose(outputs[-1].points[0].positions,
                    np.array([.1,.1,value,-.3])+.00144*protected_action,atol=1e-8)
        finally:
            process.terminate();process.wait(timeout=5)
            actor.close();executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)


def test_monitor_rejection_never_cancels_or_commands_a_controller(tmp_path):
    model=ROOT.parent/'omx_turtlebot3_policy/models/omx_turtlebot3_v6'
    executable=Path(get_package_prefix('omx_turtlebot3_control_cpp'))/'lib/omx_turtlebot3_control_cpp/ros_policy_relay_cpp'
    context=Context();rclpy.init(context=context,domain_id=228)
    node=Node('relay_monitor_test',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    joints=node.create_publisher(JointState,'/joint_states',10)
    observations=node.create_publisher(Float32MultiArray,'/omx_v6/observation',10)
    references=node.create_publisher(JointTrajectory,'/omx_v6/nominal_joint_trajectory',10)
    commands=[];cancels=[]
    subscriptions=[node.create_subscription(JointTrajectory,'/isolated_monitor_command',lambda m:commands.append(m),10),
                   node.create_subscription(Bool,'/mp_control/cancel',lambda m:cancels.append(m),10)]
    env=dict(os.environ,ROS_DOMAIN_ID='228',ROS_LOG_DIR=str(tmp_path/'ros'))
    with (tmp_path/'monitor.log').open('w') as log:
        process=subprocess.Popen([str(executable),'--ros-args','-p','model_dir:='+str(model),
            '-p','execute:=false','-p','output_topic:=/isolated_monitor_command'],env=env,stdout=log,stderr=subprocess.STDOUT)
        start=time.monotonic();sent=False
        try:
            while time.monotonic()-start<2:
                assert process.poll() is None
                joints.publish(JointState(name=['joint1','joint2','joint3','joint4'],position=[0.]*4))
                observations.publish(Float32MultiArray(data=[0.]*69))
                executor.spin_once(timeout_sec=.02)
                if time.monotonic()-start>.8 and not sent and references.get_subscription_count()>0:
                    point=JointTrajectoryPoint(positions=[100.,0.,0.,0.]);point.time_from_start.sec=1
                    references.publish(JointTrajectory(joint_names=['joint1','joint2','joint3','joint4'],points=[point]));sent=True
            assert sent and 'Reference outside mp_control motor-interface limits' in (tmp_path/'monitor.log').read_text()
            assert commands==[] and cancels==[]
        finally:
            process.terminate();process.wait(timeout=5)
            executor.shutdown();node.destroy_node();rclpy.shutdown(context=context)
