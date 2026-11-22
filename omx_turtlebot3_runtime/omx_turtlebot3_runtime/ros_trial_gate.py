"""One operator-requested mission after measured Stay and live policy inputs."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

STAY=(.104311,.027612,-.001534,-1.638291)


def at_stay(names, positions, velocities):
    try:
        index=[names.index(f'joint{i}') for i in range(1,5)]
        return all(abs(positions[j]-q)<=.08 and abs(velocities[j])<=.03
                   for j,q in zip(index,STAY))
    except (ValueError,IndexError):
        return False


class TrialGate(Node):
    def __init__(self,args):
        super().__init__('omx_v6_trial_gate')
        self.args=args;self.started=False;self.completed=False;self.fault=None
        self.inputs={'observer':(False,0.),'policy':(False,0.)}
        self.stay=(False,0.);self.patched=False;self.controllers=False
        self.since=None;self.requested=time.monotonic();self.last_query=-1e30
        args.output.parent.mkdir(parents=True,exist_ok=True)
        self.log=args.output.open('x',buffering=1)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.start_pub=self.create_publisher(Bool,'/mp_control/start',10)
        self.cancel_pub=self.create_publisher(Bool,'/mp_control/cancel',10)
        self.policy_cancel=self.create_publisher(Bool,'/omx_v6/policy_cancel',10)
        self.create_subscription(Bool,'/omx_v6/observer_ready',self.observer,qos)
        self.create_subscription(Bool,'/omx_v6/policy_ready',self.policy,qos)
        self.create_subscription(JointState,'/joint_states',self.joints,10)
        self.create_subscription(String,'/mp_control/status',self.status,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.parameters=self.create_client(GetParameters,'/mp_control_node/get_parameters')
        self.list_controllers=self.create_client(ListControllers,'/controller_manager/list_controllers')
        self.emit('WAITING_FOR_STAY',execute=args.execute,selected_id=0)

    def emit(self,event,**payload):
        value=json.dumps(dict(event=event,monotonic=time.monotonic(),**payload),allow_nan=False)
        self.log.write(value+'\n');self.get_logger().info(value)

    def observer(self,msg):self.inputs['observer']=(bool(msg.data),time.monotonic())
    def policy(self,msg):self.inputs['policy']=(bool(msg.data),time.monotonic())
    def joints(self,msg):self.stay=(at_stay(msg.name,msg.position,msg.velocity),time.monotonic())
    def status(self,msg):
        if self.started and 'cargo_loaded:' in msg.data:
            self.completed=True
            self.emit('CONTROLLER_SEQUENCE_COMPLETE',status=msg.data,physical_success_not_inferred=True)
        elif self.started and ('cancel requested' in msg.data or 'aborted' in msg.data.lower()):
            self.fault=msg.data

    def query(self):
        if time.monotonic()-self.last_query<1:return
        self.last_query=time.monotonic()
        if self.parameters.service_is_ready():
            request=GetParameters.Request(names=['hold_joint4_during_extension'])
            def parameters_done(future):
                try:
                    values=future.result().values
                    self.patched=len(values)==1 and values[0].type==1 and values[0].bool_value
                except Exception:self.patched=False
            self.parameters.call_async(request).add_done_callback(parameters_done)
        if self.list_controllers.service_is_ready():
            def controllers_done(future):
                try:
                    active={c.name for c in future.result().controller if c.state=='active'}
                    self.controllers={'arm_controller','gripper_controller'}<=active
                except Exception:self.controllers=False
            self.list_controllers.call_async(ListControllers.Request()).add_done_callback(controllers_done)

    def step(self):
        now=time.monotonic()
        if self.completed or self.fault:return True
        self.query()
        if self.started:
            if now-self.requested>self.args.mission_timeout:self.fault='Mission timeout'
            if not self.controllers or not self.patched:self.fault='Required controller or joint4 hold became unavailable'
            ready,stamp=self.inputs['policy']
            if not ready or now-stamp>.6:self.fault='C++ policy input or relay expired'
            return bool(self.fault)
        ready=(self.patched and self.controllers and self.start_pub.get_subscription_count()>0
               and self.stay[0] and now-self.stay[1]<.2
               and all(flag and now-stamp<.5 for flag,stamp in self.inputs.values()))
        if not ready:self.since=None
        elif self.since is None:self.since=now
        elif now-self.since>=.3:
            if not self.args.execute:
                self.completed=True;self.emit('MONITOR_READY',motor_commands_sent=False);return True
            self.started=True;self.requested=now
            self.start_pub.publish(Bool(data=True));self.emit('MISSION_START_SENT',selected_id=0)
        if not self.started and now-self.requested>self.args.startup_timeout:
            self.fault='Startup requires measured Stay, both marker IDs, patched wrist hold, policy and active controllers'
        return bool(self.fault)

    def finish(self):
        if self.started and not self.completed:
            self.policy_cancel.publish(Bool(data=True));self.cancel_pub.publish(Bool(data=True))
            self.emit('MISSION_STOPPED',reason=self.fault or 'Operator shutdown',gripper_not_released=True)
            until=time.monotonic()+.2
            while rclpy.ok() and time.monotonic()<until:rclpy.spin_once(self,timeout_sec=.02)
        elif self.fault:self.emit('STARTUP_BLOCKED',reason=self.fault,motor_commands_sent=False)
        self.log.close();self.destroy_node()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--execute',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--startup-timeout',type=float,default=45.)
    parser.add_argument('--mission-timeout',type=float,default=300.)
    args,ros_args=parser.parse_known_args()
    if not 1<=args.startup_timeout<=120 or not 1<=args.mission_timeout<=600:parser.error('Invalid timeout')
    rclpy.init(args=ros_args);node=TrialGate(args)
    try:
        while rclpy.ok() and not node.step():rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        failed=bool(node.fault);node.finish()
        if rclpy.ok():rclpy.shutdown()
    if failed:raise SystemExit(1)

if __name__=='__main__':main()
