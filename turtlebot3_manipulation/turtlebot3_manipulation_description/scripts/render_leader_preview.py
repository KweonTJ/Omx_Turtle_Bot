#!/usr/bin/env python3
"""Render URDF visuals offline, without ROS publishers, hardware, or simulation.

Requires numpy, trimesh, mujoco and Pillow. This is a static visual QA renderer,
not a physics conversion. It consumes the resolved URDF rather than a second
hand-written robot scene.
"""
import argparse
import os
from pathlib import Path
import xml.etree.ElementTree as ET

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np
from PIL import Image, ImageDraw
import trimesh.transformations as tf


def origin(element):
    node = element.find('origin')
    if node is None:
        return np.eye(4)
    matrix = tf.euler_matrix(*map(float,node.get('rpy','0 0 0').split()))
    matrix[:3,3] = list(map(float,node.get('xyz','0 0 0').split()))
    return matrix


def build_scene(urdf, package, positions, mode='overview'):
    root = ET.parse(urdf).getroot()
    joints = root.findall('joint')
    children = {j.find('child').get('link') for j in joints}
    frames = {link.get('name'):np.eye(4) for link in root.findall('link')
              if link.get('name') not in children}
    pending = list(joints)
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in frames]
        if not ready:
            raise ValueError('Cyclic or disconnected URDF')
        for joint in ready:
            motion = np.eye(4)
            mimic = joint.find('mimic')
            q = positions.get(joint.get('name'),0)
            if mimic is not None:
                q = positions.get(mimic.get('joint'),0)*float(mimic.get('multiplier',1))+float(mimic.get('offset',0))
            if joint.get('type') != 'fixed':
                axis = np.array(list(map(float,joint.find('axis').get('xyz').split())))
                if joint.get('type') == 'prismatic':
                    motion[:3,3] = axis*q
                else:
                    motion = tf.rotation_matrix(q,axis)
            frames[joint.find('child').get('link')] = frames[joint.find('parent').get('link')]@origin(joint)@motion
            pending.remove(joint)

    scene = ET.Element('mujoco',model='leader_visual_review')
    ET.SubElement(scene,'compiler',angle='radian')
    visual = ET.SubElement(scene,'visual')
    ET.SubElement(visual,'global',offwidth='1800',offheight='1200')
    ET.SubElement(visual,'headlight',ambient='0.25 0.25 0.25',diffuse='0.5 0.5 0.5',specular='0.1 0.1 0.1')
    asset, world = ET.SubElement(scene,'asset'),ET.SubElement(scene,'worldbody')
    ET.SubElement(asset,'texture',type='skybox',builtin='flat',rgb1='.85 .88 .92',
                  rgb2='.85 .88 .92',width='32',height='192')
    ET.SubElement(world,'light',pos='1 -1 2',dir='-1 1 -2',diffuse='0.35 0.35 0.35',specular='0.1 0.1 0.1')
    ET.SubElement(world,'geom',type='plane',size='2 2 .01',rgba='.86 .88 .91 1')
    colors = {m.get('name'):m.find('color').get('rgba') for m in root.findall('material') if m.find('color') is not None}
    textured = {}
    for m in root.findall('material'):
        texture = m.find('texture')
        if texture is None:
            continue
        path = package/texture.get('filename').split('/',3)[3]
        name = m.get('name')
        ET.SubElement(asset,'texture',name=name,type='2d',file=str(path))
        ET.SubElement(asset,'material',name=name,texture=name,texuniform='false',
                      specular='0.12',shininess='0.1')
        textured[name] = name
    mesh_assets = {}
    for link in root.findall('link'):
        name=link.get('name')
        owner=name.split('__visual_',1)[0]
        drive=name.startswith(('motor_','sprocket_','track_'))
        if mode=='drivetrain' and not drive:
            continue
        if mode=='chassis' and not (drive or owner in ('base_link','opencr_link')):
            continue
        if mode=='plate' and name!='base_link':
            continue
        if mode=='electronics' and owner not in ('opencr_link','jetson_orin_nano_link'):
            continue
        if mode in ('opencr_link','jetson_orin_nano_link') and owner != mode:
            continue
        for v in link.findall('visual'):
            if mode=='plate' and v is not link.findall('visual')[0]:
                continue
            mat = frames[link.get('name')]@origin(v)
            attrs = dict(pos=' '.join(map(str,mat[:3,3])),
                         quat=' '.join(map(str,tf.quaternion_from_matrix(mat))),
                         contype='0',conaffinity='0')
            material = v.find('material')
            color = material.find('color') if material is not None else None
            attrs['rgba'] = color.get('rgba') if color is not None else colors.get(material.get('name') if material is not None else '', '.4 .4 .4 1')
            if material is not None and material.get('name') in textured:
                attrs.update(material=material.get('name'),rgba='1 1 1 1')
            geom = v.find('geometry')[0]
            if geom.tag == 'mesh':
                uri = geom.get('filename')
                if uri.startswith('package://turtlebot3_manipulation_description/'):
                    path = package/uri.split('/',3)[3]
                else:
                    path = Path(uri.removeprefix('file://'))
                key = (str(path),geom.get('scale','1 1 1'))
                if key not in mesh_assets:
                    name = f'mesh_{len(mesh_assets)}'
                    ET.SubElement(asset,'mesh',name=name,file=str(path),scale=key[1])
                    mesh_assets[key] = name
                attrs.update(type='mesh',mesh=mesh_assets[key])
            elif geom.tag == 'box':
                attrs.update(type='box',size=' '.join(str(float(x)/2) for x in geom.get('size').split()))
            elif geom.tag == 'cylinder':
                attrs.update(type='cylinder',size=f"{geom.get('radius')} {float(geom.get('length'))/2}")
            elif geom.tag == 'sphere':
                attrs.update(type='sphere',size=geom.get('radius'))
            else:
                raise ValueError(f'Unsupported geometry {geom.tag}')
            ET.SubElement(world,'geom',**attrs)
    return ET.tostring(scene,encoding='unicode')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf',type=Path)
    parser.add_argument('--package',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--detail',action='store_true',help='Plate and drivetrain inspection views')
    parser.add_argument('--electronics',action='store_true',help='Board closeups with occluding plates hidden')
    args = parser.parse_args()
    # Display pose only, chosen to expose the folded arm and camera seen in video.
    positions = dict(joint1=0,joint2=-1.15,joint3=.60,joint4=.55,gripper_left_joint=.010)
    width,height = 800,700
    sheet = Image.new('RGB',(width*2,height*2+90),'#e5e9ee')
    draw = ImageDraw.Draw(sheet)
    draw.text((25,15),'LEADER TURTLEBOT3 | 2026-10-13 video reconstruction',fill='#172b40')
    draw.text((25,40),'L2: OpenCR | L3: Jetson Orin Nano | Arm on rear half plate | Inspection colors + PCB textures',fill='#42566b')
    views = [('overview',140,-18,'FRONT / LEFT'),('overview',90,-10,'SIDE'),
             ('overview',180,-12,'FRONT'),('overview',-45,-28,'REAR / RIGHT')]
    if args.detail:
        views=[('plate',0,-90,'PLATE / TOP: FRONT (+X) above / missing rear half below'),
               ('drivetrain',0,-90,'4 MOTOR BODIES / TOP: plates hidden'),
               ('chassis',-40,-25,'LOWER CHASSIS: upper tower hidden'),
               ('drivetrain',140,-35,'MOTORS > OUTPUT HUBS > SPROCKETS: plates hidden')]
    if args.electronics:
        views=[('opencr_link',0,-65,'L2 / OpenCR 1.0: illustrative components and silkscreen'),
               ('jetson_orin_nano_link',-45,-50,'L3 / Jetson Orin Nano: illustrative cooler and ports'),
               ('electronics',90,-12,'TWO FLOORS: plates hidden for inspection only'),
               ('overview',-40,-15,'ASSEMBLED / REAR: electronics inside original stack')]
    for i,(mode,azim,elev,title) in enumerate(views):
        model = mujoco.MjModel.from_xml_string(build_scene(args.urdf,args.package,positions,mode))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model,data)
        with mujoco.Renderer(model,height=height,width=width) as renderer:
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [-.064,0,.04 if args.detail else .21]
            camera.distance = .49 if args.detail else .85
            if args.electronics:
                camera.lookat[:] = ([-.032,0,.085] if mode=='opencr_link' else
                                   [-.09,0,.15] if mode=='jetson_orin_nano_link' else
                                   [-.06,0,.12] if mode=='electronics' else [-.064,0,.21])
                camera.distance = .25 if mode.endswith('_link') else .4 if mode=='electronics' else .85
            camera.azimuth,camera.elevation = azim,elev
            renderer.update_scene(data,camera=camera)
            tile = Image.fromarray(renderer.render())
            ImageDraw.Draw(tile).text((18,18),title,fill='#172b40')
            sheet.paste(tile,((i%2)*width,90+(i//2)*height))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    sheet.save(args.output)
    print(args.output)


if __name__ == '__main__':
    main()
