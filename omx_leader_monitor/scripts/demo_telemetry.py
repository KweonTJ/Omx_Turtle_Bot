#!/usr/bin/env python3
"""Synthetic display review ONLY. Publishes exclusively /leader_monitor/demo/*.

Not inference, not a pick-place simulation result, never hardware joint_states.
"""
import argparse
import io
import json
import math
import time

from PIL import Image as PILImage, ImageDraw
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, Image, CompressedImage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool


class Demo(Node):
    def __init__(self, transport):
        super().__init__('leader_monitor_demo')
        prefix = '/leader_monitor/demo'
        self.transport = transport
        self.start = time.monotonic()
        self.joints = self.create_publisher(JointState, prefix + '/joint_states', 5)
        self.status = self.create_publisher(String, prefix + '/status', 5)
        self.request = self.create_publisher(String, prefix + '/host_request',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.info = self.create_publisher(String, prefix + '/target_info', 5)
        self.pose = self.create_publisher(PoseStamped, prefix + '/aruco_pose', 5)
        self.valid = self.create_publisher(Bool, prefix + '/target_valid', 5)
        suffix = '/compressed' if transport == 'compressed' else ''
        self.camera = self.create_publisher(CompressedImage if suffix else Image, prefix + '/eef/image_raw' + suffix, 2)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.1, self.image)
        print('DEMO: synthetic telemetry, private topics only; no real robot commands', flush=True)

    def values(self):
        t = time.monotonic() - self.start
        phase = t % 36
        distance = 0.48 - 0.35 * min(1.0, max(0.0, (phase - 3) / 11))
        if phase > 24:
            distance = 0.13 + 0.3 * (phase - 24) / 12
        stages = [(3, 'SEARCH_TARGET'), (14, 'PICK_REACH'), (18, 'CLOSE_GRIPPER'),
                  (25, 'PLACE_REACH'), (29, 'OPEN_RELEASE'), (33, 'PLACE_TO_STAY'), (36, 'COMPLETE')]
        state = next(name for end, name in stages if phase < end)
        return t, phase, distance, state

    def tick(self):
        t, phase, distance, state = self.values()
        stamp = self.get_clock().now().to_msg()
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint', 'wheel_left_joint', 'wheel_right_joint']
        q2, q3 = -0.95 + 0.25 * math.sin(t / 3), 0.5 + 0.18 * math.sin(t / 3 + 0.5)
        joint.position = [0.35 * math.sin(t / 6), q2, q3, -q2-q3, 0.003 if 15 < phase < 27 else 0.012, 0.0, 0.0]
        self.joints.publish(joint)
        self.status.publish(String(data=f'state={state} detail=DEMO synthetic scene; not an RL result'))
        self.request.publish(String(data=json.dumps({'request_id': f'demo-{int(t // 36) + 1:03}', 'object_id': 3, 'command': 'PICK_PLACE'})))
        self.valid.publish(Bool(data=True))
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'eef_usb_camera_optical_frame'
        pose.pose.position.z = distance
        pose.pose.orientation.w = 1.0
        self.pose.publish(pose)
        self.info.publish(String(data=json.dumps({'marker_id': 3, 'valid': True, 'shelf_level': 2,
                                                  'frame_id': 'base_link', 'position': [0.24, -0.04, 0.26]})))

    def image(self):
        t, phase, distance, state = self.values()
        image = PILImage.new('RGB', (640, 360), (43, 51, 65))
        draw = ImageDraw.Draw(image)
        draw.rectangle((12, 45, 628, 320), fill=(147, 154, 159), outline=(224, 230, 233), width=6)
        draw.line((14, 210, 626, 210), fill=(227, 232, 235), width=10)
        for x in range(30, 615, 18):
            draw.rounded_rectangle((x, 56, x+6, 190), radius=3, fill=(67, 73, 83))
        for x, y, number in ((110, 170, 1), (470, 172, 2), (300, 270, 4)):
            draw.rectangle((x-23, y-32, x+23, y+5), fill=(190, 144, 86), outline='white')
            draw.text((x-5, y-20), str(number), fill='black')
        size = int(min(145, 32 / distance))
        cx, cy = 320 + int(15 * math.sin(t/5)), 160
        draw.rectangle((cx-size, cy-size//2, cx+size, cy+size), fill=(79, 159, 156), outline=(67, 237, 213), width=3)
        # Illustrative marker-like graphic only: intentionally NOT a valid ArUco target.
        half = size // 2
        draw.rectangle((cx-half, cy-half, cx+half, cy+half), fill='white')
        draw.rectangle((cx-half+5, cy-half+5, cx+half-5, cy+half-5), fill='black')
        draw.text((cx-3, cy-6), '3', fill='white')
        draw.text((18, 12), 'SYNTHETIC EEF CAMERA / DEMO ONLY', fill=(255, 206, 114))
        draw.text((18, 335), f'Target ID 3 | {distance:.2f} m | {state}', fill='white')
        if self.transport == 'compressed':
            output = io.BytesIO(); image.save(output, format='JPEG', quality=85)
            msg = CompressedImage(); msg.format = 'jpeg'; msg.data = output.getvalue()
        else:
            msg = Image(); msg.width = 640; msg.height = 360; msg.step = 1920; msg.encoding = 'rgb8'; msg.data = image.tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'eef_usb_camera_optical_frame'
        self.camera.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--transport', choices=['raw', 'compressed'], default='raw')
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = Demo(args.transport)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
