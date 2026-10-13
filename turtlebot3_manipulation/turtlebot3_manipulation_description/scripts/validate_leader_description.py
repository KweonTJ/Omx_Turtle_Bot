#!/usr/bin/env python3
"""Offline regression checks: xacro expansion, meshes, tree, legacy TF parity."""
from pathlib import Path
import itertools
import copy
import math
import subprocess
import struct
import tempfile
import xml.etree.ElementTree as ET
from export_leader_urdf import split_visual_materials


PACKAGE = Path(__file__).resolve().parents[1]


def expanded(profile, args):
    xml = subprocess.check_output(['xacro',str(PACKAGE/'xacro'/profile),*args],text=True)
    return xml,ET.fromstring(xml)


def signature(element):
    return (element.tag,sorted(element.attrib.items()),(element.text or '').strip(),
            [signature(child) for child in element])


def values(node, attribute, default='0 0 0'):
    return list(map(float,node.get(attribute,default).split()))


def primitive_bounds(shape, translation=(0,0,0)):
    origin=shape.find('origin')
    center=[a+b for a,b in zip(translation,values(origin,'xyz'))]
    rotation=values(origin,'rpy')
    assert rotation[:2]==[0,0], 'Electronics check expects only in-plane rotations'
    geo=shape.find('geometry')[0]
    if geo.tag=='box':
        half=[v/2 for v in values(geo,'size')]
        c,s=abs(math.cos(rotation[2])),abs(math.sin(rotation[2]))
        half=[c*half[0]+s*half[1],s*half[0]+c*half[1],half[2]]
    else:
        assert geo.tag=='cylinder'
        half=[float(geo.get('radius'))]*2+[float(geo.get('length'))/2]
    return [(v-h,v+h) for v,h in zip(center,half)]


def separated(a,b):
    return any(hi<=other_lo+1e-9 or other_hi<=lo+1e-9
               for (lo,hi),(other_lo,other_hi) in zip(a,b))


def check_rviz_compatibility(original):
    converted=split_visual_materials(copy.deepcopy(original))
    assert signature(converted)==signature(split_visual_materials(copy.deepcopy(converted))), 'Not idempotent'
    for joint in original.findall('joint'):
        assert signature(joint)==signature(converted.find(f"joint[@name='{joint.get('name')}']"))
    for original_link in original.findall('link'):
        name=original_link.get('name')
        actual=converted.find(f"link[@name='{name}']")
        assert [signature(c) for c in original_link if c.tag!='visual']==[signature(c) for c in actual if c.tag!='visual']
        visuals=list(actual.findall('visual'))
        for child in converted.findall('link'):
            if child.get('name').startswith(name+'__visual_'):
                joint=converted.find(f"joint[@name='{child.get('name')}_fixed']")
                assert joint.get('type')=='fixed' and joint.find('parent').get('link')==name
                assert values(joint.find('origin'),'xyz')==[0,0,0] and values(joint.find('origin'),'rpy')==[0,0,0]
                visuals.extend(child.findall('visual'))
        assert sorted(map(str,map(signature,visuals)))==sorted(map(str,map(signature,original_link.findall('visual')))), 'Visual geometry/material drift'
    for link in converted.findall('link'):
        materials={str(signature(v.find('material'))) for v in link.findall('visual')}
        assert len(materials)<=1, 'Humble RViz would flatten multiple materials'
    return ET.tostring(converted,encoding='unicode')


def check_electronics(root,prefix):
    specifications=(('opencr',[-.032,0,.05775],(.105,.075,.018),.11275,'base_link'),
                    ('jetson_orin_nano',[-.09,0,.12175],(.100,.079,.035),.17675,'camera_plate_link'))
    obstacles=[]
    base=root.find(f"link[@name='{prefix}base_link']")
    assert not any(v.find('material').get('name')=='leader_battery' for v in base.findall('visual')), 'Removed blue battery placeholder reappeared'
    camera=root.find("link[@name='camera_astra_visual_frame']")
    camera_z=values(root.find(f"joint[@name='{prefix}base_to_camera_plate_joint']/origin"),'xyz')[2]+.0025
    obstacles.extend(primitive_bounds(v,(.045,0,camera_z)) for v in camera.findall('collision'))
    for link in root.findall('link'):
        if link.get('name').startswith(prefix+'support_'):
            joint=root.find(f"joint/child[@link='{link.get('name')}']/..")
            obstacles.append(primitive_bounds(link.find('collision'),values(joint.find('origin'),'xyz')))
    # Lower/middle support cylinders are visuals within base_link.
    obstacles.extend(primitive_bounds(v) for v in base.findall('visual') if v.find('geometry/cylinder') is not None)
    for name,base_origin,size,ceiling,parent in specifications:
        link=root.find(f"link[@name='{prefix}{name}_link']")
        joint=root.find(f"joint[@name='{prefix}{name}_mount']")
        assert joint.get('type')=='fixed' and joint.find('parent').get('link')==prefix+parent
        local=values(joint.find('origin'),'xyz')
        if parent=='camera_plate_link':
            plate=values(root.find(f"joint[@name='{prefix}base_to_camera_plate_joint']/origin"),'xyz')
            local=[a+b for a,b in zip(local,plate)]
        assert all(abs(a-b)<1e-9 for a,b in zip(local,base_origin))
        assert values(link.find('visual/geometry/box'),'size')==[size[0],size[1],.0016]
        envelope=primitive_bounds(link.find('collision'),base_origin)
        assert values(link.find('collision/geometry/box'),'size')==list(size)
        assert abs(envelope[2][0]-base_origin[2])<1e-9, 'Board feet must meet own floor'
        assert envelope[2][1]<ceiling, 'Board/cooler penetrates ceiling'
        assert envelope[0][0]>=-.1647735 and envelope[0][1]<=.0367735
        assert envelope[1][0]>=-.0687735 and envelope[1][1]<=.0687735
        for visual in link.findall('visual'):
            bbox=primitive_bounds(visual,base_origin)
            assert all(lo>=el-1e-9 and hi<=eh+1e-9 for (lo,hi),(el,eh) in zip(bbox,envelope)), name
        assert all(separated(envelope,obstacle) for obstacle in obstacles), f'{name} overlaps camera/posts/board'
        obstacles.append(envelope)
    # Palette regressions: STL visuals must no longer all share leader_black.
    expected={'base_link':'leader_plate','track_left_link':'leader_rubber',
              'motor_front_left_link':'leader_motor','sprocket_front_left_link':'leader_sprocket',
              'link1':'leader_arm','opencr_link':'leader_opencr_surface',
              'jetson_orin_nano_link':'leader_jetson_surface'}
    assert len(set(expected.values()))==len(expected)
    for name,material in expected.items():
        assert root.find(f"link[@name='{prefix}{name}']/visual/material").get('name')==material
    for material in ('leader_opencr_surface','leader_jetson_surface'):
        texture=root.find(f"material[@name='{material}']/texture")
        uri=texture.get('filename')
        assert uri.startswith('package://turtlebot3_manipulation_description/')
        payload=(PACKAGE/uri.split('/',3)[3]).read_bytes()
        assert payload[:8]==b'\x89PNG\r\n\x1a\n' and struct.unpack_from('>II',payload,16)==(1024,768)


def check_support_stack(root,prefix):
    """Measured support lengths, seated between the existing 9 mm plates."""
    base=root.find(f"link[@name='{prefix}base_link']")
    lower_plates=[v for v in base.findall('visual') if v.find('geometry/mesh') is not None]
    assert len(lower_plates)==2
    plate_z=[values(v.find('origin'),'xyz')[2] for v in lower_plates]
    plate_z.extend(values(root.find(f"joint[@name='{prefix}{name}']/origin"),'xyz')[2]
                   for name in ('base_to_camera_plate_joint','upper_waffle_plate_joint'))
    assert all(abs(a-b)<1e-9 for a,b in zip(plate_z,[.00425,.05325,.11725,.18125]))
    supports=[primitive_bounds(v) for v in base.findall('visual') if v.find('geometry/cylinder') is not None]
    upper=[]
    for end,side in itertools.product(('front','back'),('left','right')):
        name=f'{prefix}support_{end}_{side}'
        link=root.find(f"link[@name='{name}_link']")
        origin=values(root.find(f"joint[@name='{name}_joint']/origin"),'xyz')
        visual=primitive_bounds(link.find('visual'),origin)
        assert visual==primitive_bounds(link.find('collision'),origin)
        upper.append(visual)
    supports.extend(upper)
    assert len(supports)==12, 'Four posts on each of three floors'
    for bottom,top,length in zip(plate_z,plate_z[1:],[.040,.055,.055]):
        floor,ceiling=bottom+.0045,top-.0045
        assert abs(ceiling-floor-length)<1e-9, 'Support length was treated as centre spacing'
        seated=[b for b in supports if abs(b[2][0]-floor)<1e-9 and abs(b[2][1]-ceiling)<1e-9]
        assert len(seated)==4, 'Support-to-plate gap or penetration'
    arm_z=values(root.find(f"joint[@name='{prefix}base_fixed']/origin"),'xyz')[2]
    assert abs(arm_z-plate_z[-1]-.0005)<1e-9, 'Arm did not follow its mounting plate'


def main():
    plate_files = [PACKAGE/'meshes/leader_video'/name for name in
                   ('lower_plate.stl','camera_plate.stl','upper_plate.stl')]
    payloads = [path.read_bytes() for path in plate_files]
    assert payloads[0] == payloads[1] == payloads[2], 'Every layer must use the same assembly'
    count = struct.unpack_from('<I',payloads[0],80)[0]
    assert count == 3*2582, 'Each layer must contain exactly three source half plates'
    triangles = list(struct.iter_unpack('<12fH',payloads[0][84:]))
    vertices = [triangle[start:start+3] for triangle in triangles for start in (3,6,9)]
    bounds = [(min(v[i] for v in vertices),max(v[i] for v in vertices)) for i in range(3)]
    for extent, expected in zip((hi-lo for lo,hi in bounds),(.201547,.137547,.009)):
        assert abs(extent-expected)<5e-6, 'Source plate geometry was stretched'
    # Verify each original half's placement, not only the assembly bounding box:
    # a wrong three-parallel-halves assembly has the SAME overall dimensions.
    expected_bounds=[((-.0367735,.1007735),(-.0687735,.0047735)),
                     ((-.0367735,.1007735),(-.0047735,.0687735)),
                     ((-.1007735,-.0272265),(-.0687735,.0687735))]
    for i,expected in enumerate(expected_bounds):
        half=vertices[i*2582*3:(i+1)*2582*3]
        for axis,pair in enumerate(expected):
            actual=(min(v[axis] for v in half),max(v[axis] for v in half))
            assert all(abs(a-b)<5e-6 for a,b in zip(actual,pair)), 'Half-plate orientation/layout regression'
    print('PASS: every plate = front octagon (2 opposing halves) + rear half, rear end absent')
    payload=(PACKAGE/'meshes/leader_video/track.stl').read_bytes()
    assert struct.unpack_from('<I',payload,80)[0]==41*5*12, 'Belt visual subdivision drift'
    belt_vertices=[row[i:i+3] for row in struct.iter_unpack('<12fH',payload[84:]) for i in (3,6,9)]
    belt_bounds=[(min(v[i] for v in belt_vertices),max(v[i] for v in belt_vertices)) for i in range(3)]
    assert .215 < belt_bounds[0][1]-belt_bounds[0][0] < .2175, 'Belt does not span measured-case-gap axle spacing'
    assert abs(sum(belt_bounds[0])/2+.06925)<.0002, 'Belt not centred on axle pair'
    assert all(lo>=lower-1e-7 and hi<=upper+1e-7 for (lo,hi),(lower,upper) in
               zip(belt_bounds,[(-.178,.0395),(-.021,.021),(-.0395,.0395)])), 'Belt outside new collision envelope'
    motor_z_bounds={}
    motor_x_bounds={}
    motor_y_bounds={}
    for side in ('left','right'):
        motor=PACKAGE/f'meshes/leader_video/motor_{side}.stl'
        payload=motor.read_bytes()
        assert struct.unpack_from('<I',payload,80)[0]==1492
        zs=[row[i] for row in struct.iter_unpack('<12fH',payload[84:]) for i in (5,8,11)]
        motor_z_bounds[side]=(min(zs),max(zs))
        xs=[row[i] for row in struct.iter_unpack('<12fH',payload[84:]) for i in (3,6,9)]
        motor_x_bounds[side]=(min(xs),max(xs))
        ys=[row[i] for row in struct.iter_unpack('<12fH',payload[84:]) for i in (4,7,10)]
        motor_y_bounds[side]=(min(ys),max(ys))
        assert abs((min(ys) if side=='left' else max(ys))-(-.045 if side=='left' else .045))<1e-7
        assert abs(min(zs)+.01425)<1e-7 and abs(max(zs)-.01425)<1e-7, 'Motor mesh not centred on its actual shaft'
    with tempfile.TemporaryDirectory(prefix='leader_urdf_validate_') as temp:
        for prefix, camera, camera_tf in itertools.product(['','leader_'],['true','false'],['true','false']):
            args = [f'prefix:={prefix}',f'use_eef_usb_camera:={camera}',f'use_camera_driver_tf:={camera_tf}']
            _,old = expanded('turtlebot3_manipulation.urdf.xacro',args)
            xml,new = expanded('leader_turtlebot3.urdf.xacro',args)
            # Display-only changes: rear-half arm mount and measured stack.
            # Only these three origins may differ; all relative sensor/arm
            # extrinsics and every other legacy joint must remain identical.
            expected=copy.deepcopy(old)
            permitted={'base_fixed':[-.128,0,.18175],
                       'base_to_camera_plate_joint':[-.064,0,.11725],
                       'upper_waffle_plate_joint':[-.092,0,.18125]}
            hardware={'base_fixed':[-.092,0,.178],
                      'base_to_camera_plate_joint':[-.064,0,.115],
                      'upper_waffle_plate_joint':[-.092,0,.1775]}
            for name,xyz in permitted.items():
                actual=new.find(f"joint[@name='{prefix}{name}']/origin")
                assert all(abs(a-b)<1e-9 for a,b in zip(values(actual,'xyz'),xyz)), name
                assert values(old.find(f"joint[@name='{prefix}{name}']/origin"),'xyz')==hardware[name], 'Hardware stack changed'
                expected.find(f"joint[@name='{prefix}{name}']/origin").set('xyz',actual.get('xyz'))
            old_joints = {j.get('name'):signature(j) for j in expected.findall('joint')}
            new_joints = {j.get('name'):signature(j) for j in new.findall('joint')}
            assert all(new_joints.get(name) == value for name,value in old_joints.items()), 'Legacy joint/TF drift'
            assert len(new_joints) == len(old_joints)+12, '2 belts + 4 motor mounts + 4 wheel rotors + 2 boards'
            assert len({link.get('name') for link in new.findall('link')}) == len(new.findall('link'))
            assert new.find('ros2_control') is None, 'Hardware plugin in display profile'
            assert not new.findall('.//gazebo/plugin'), 'Gazebo control plugin in display profile'
            check_electronics(new,prefix)
            check_support_stack(new,prefix)
            mount = new.find(f"joint[@name='{prefix}base_fixed']/origin")
            assert all(abs(a-b)<1e-9 for a,b in zip(values(mount,'xyz'),[-.128,0,.18175])), 'Rear-half mount / measured tower height drift'
            assert list(map(float,mount.get('rpy').split())) == [0,0,0], 'Standard mounting direction drift'
            assert values(old.find(f"joint[@name='{prefix}base_fixed']/origin"),'xyz')==[-.092,0,.178], 'Hardware mount changed'
            # Source link1 bottom footprint is X +/-23.25, Y +/-14.25 mm.
            rear_lo,rear_hi=(x-.064 for x in expected_bounds[2][0])
            assert rear_lo<-.128-.02325 and -.128+.02325<rear_hi, 'Arm base extends off rear half'
            plate_visuals = [mesh for mesh in new.findall('.//visual/geometry/mesh')
                             if mesh.get('filename').rsplit('/',1)[-1] in
                             ('lower_plate.stl','camera_plate.stl','upper_plate.stl')]
            assert len(plate_visuals) == 4, 'All four levels must use the corrected plates'
            assert all(mesh.get('scale','1 1 1') == '1 1 1' for mesh in plate_visuals)
            base=new.find(f"link[@name='{prefix}base_link']")
            floor=base.findall('visual')[0]
            floor_z=float(floor.find('origin').get('xyz').split()[2])
            floor_top=floor_z+bounds[2][1]
            assert abs(floor_z-.00425)<1e-9
            floor_collision=base.find("collision[@name='leader_floor_plate_collision']")
            collision_top=float(floor_collision.find('origin').get('xyz').split()[2])+float(floor_collision.find('geometry/box').get('size').split()[2])/2
            assert abs(collision_top-floor_top)<1e-7, 'Floor visual/collision mismatch'
            for name in ('camera_plate_link','upper_waffle_plate'):
                box = new.find(f"link[@name='{prefix}{name}']/collision/geometry/box")
                assert box.get('size') == '0.20155 0.13755 0.009'
            # Check every body-camera housing, lens and pedestal, including
            # collisions. Uniform display enlargement; relative extrinsics unchanged.
            camera_body=new.find("link[@name='camera_astra_visual_frame']")
            assert camera_body is not None
            fronts=[]
            for shape in camera_body.findall('visual')+camera_body.findall('collision'):
                x=float(shape.find('origin').get('xyz').split()[0])
                geometry=shape.find('geometry')[0]
                if geometry.tag=='box':
                    half_x=float(geometry.get('size').split()[0])/2
                else:
                    assert geometry.tag=='cylinder'
                    rpy=list(map(float,shape.find('origin').get('rpy').split()))
                    assert abs(rpy[1]-1.5707963267948966)<1e-9
                    half_x=float(geometry.get('length'))/2
                fronts.append(.109+x+half_x)  # camera-plate local coordinates
            assert len(fronts)==9
            assert all(front <= bounds[0][1] for front in fronts), 'Body camera protrudes forward of plate'
            assert abs(max(fronts)-.0995)<1e-9, 'Camera display recess drift'
            scale=.055/.0535
            assert all(abs(a-b*scale)<1e-9 for a,b in zip(values(camera_body.find('visual/geometry/box'),'size'),[.04,.135,.03]))
            camera_collisions=camera_body.findall('collision')
            assert all(abs(a-b*scale)<1e-9 for a,b in zip(values(camera_collisions[0].find('geometry/box'),'size'),[.04,.165,.03])), 'Camera collision scale mismatch'
            visuals=camera_body.findall('visual')
            for index,radius,length,y in ((1,.015,.04,.0675),(2,.015,.04,-.0675),
                                          (4,.008,.001,.045),(5,.008,.001,-.017),(6,.005,.001,-.045)):
                shape=visuals[index]
                cylinder=shape.find('geometry/cylinder')
                assert abs(float(cylinder.get('radius'))-radius*scale)<1e-9
                assert abs(float(cylinder.get('length'))-length*scale)<1e-9
                assert abs(values(shape.find('origin'),'xyz')[1]-y*scale)<1e-9
            assert all(abs(a-b*scale)<1e-9 for a,b in zip(values(visuals[3].find('geometry/box'),'size'),[.06,.06,.0235]))
            camera_origin_z=float(new.find(f"joint[@name='{prefix}base_to_camera_plate_joint']/origin").get('xyz').split()[2])+.0025
            ceiling_z=float(new.find(f"joint[@name='{prefix}upper_waffle_plate_joint']/origin").get('xyz').split()[2])+bounds[2][0]
            camera_floor_z=camera_origin_z-.0025+bounds[2][1]
            for shapes in (camera_body.findall('visual'),camera_collisions):
                module=shapes[0]
                module_z=camera_origin_z+float(module.find('origin').get('xyz').split()[2])
                module_height=float(module.find('geometry/box').get('size').split()[2])
                assert abs(module_z+module_height/2-ceiling_z)<1e-7, 'Camera-to-ceiling gap/penetration'
                pedestal=shapes[3] if shapes[0].tag=='visual' else shapes[1]
                pedestal_z=camera_origin_z+float(pedestal.find('origin').get('xyz').split()[2])
                pedestal_height=float(pedestal.find('geometry/box').get('size').split()[2])
                assert abs(pedestal_z-pedestal_height/2-camera_floor_z)<1e-7, 'Camera pedestal floats above floor'
                assert abs(pedestal_z+pedestal_height/2-(module_z-module_height/2))<1e-7, 'Pedestal does not meet camera'
            # Recessing the camera must not push its housing through the upper
            # front posts. Check their conservative collision AABBs in base_link.
            for side in ('left','right'):
                support_name=f'{prefix}support_front_{side}'
                joint_origin=list(map(float,new.find(f"joint[@name='{support_name}_joint']/origin").get('xyz').split()))
                collision=new.find(f"link[@name='{support_name}_link']/collision")
                local=list(map(float,collision.find('origin').get('xyz').split()))
                center=[a+b for a,b in zip(joint_origin,local)]
                cylinder=collision.find('geometry/cylinder')
                half=[float(cylinder.get('radius'))]*2+[float(cylinder.get('length'))/2]
                for c in camera_body.findall('collision'):
                    camera_center=[a+b for a,b in zip((.045,0,camera_origin_z),map(float,c.find('origin').get('xyz').split()))]
                    camera_half=[float(v)/2 for v in c.find('geometry/box').get('size').split()]
                    assert any(abs(a-b)>h+k for a,b,h,k in zip(center,camera_center,half,camera_half)), 'Recessed camera intersects upper support'
            for side in ('left','right'):
                sign=1 if side=='left' else -1
                belt = new.find(f"joint[@name='{prefix}track_{side}_joint']")
                assert belt.get('type') == 'fixed'
                assert belt.find('parent').get('link') == prefix+'base_link'
                assert all(abs(a-b)<1e-9 for a,b in zip(values(belt.find('origin'),'xyz'),[0,sign*.0925,.023]))
                gap=.0925-.042/2-.137547/2
                assert abs(gap-.0027265)<1e-9, 'Measured lateral layout: plate/belt clearance drift'
                assert gap>0, 'Plate/belt overlap'
                # The measured case gap supersedes estimated hole-row alignment.
                assert .019 < .0925-.0046-.137547/2 < .020, 'Motor must still project outside plate'
                belt_link=new.find(f"link[@name='{prefix}track_{side}_link']")
                assert len(belt_link.findall('visual'))==1, 'Sprockets must not be fixed to belt'
                belt_size=values(belt_link.find('collision/geometry/box'),'size')
                assert all(abs(a-b)<1e-9 for a,b in zip(belt_size,[.2175,.042,.079]))
                assert values(belt_link.find('collision/origin'),'xyz')==[-.06925,0,0]
                wheel = new.find(f"link[@name='{prefix}wheel_{side}_link']")
                assert wheel.find('visual') is None, 'Legacy tire still visible'
                old_wheel_origin=values(old.find(f"joint[@name='{prefix}wheel_{side}_joint']/origin"),'xyz')
                assert old_wheel_origin[0]==0.0
                case_inner=motor_x_bounds[side][0]
                # Front points -X, rear is pitch=pi and therefore points +X.
                actual_case_gap=case_inner-(-.1385-case_inner)
                assert abs(actual_case_gap-.068)<1e-7, 'User-measured 68 mm case gap drift'
                for end,x in (('front',0.),('rear',-.1385)):
                    motor_name=f'{prefix}motor_{end}_{side}_link'
                    rotor_name=f'{prefix}sprocket_{end}_{side}_link'
                    mount=new.find(f"joint[@name='{prefix}motor_{end}_{side}_mount']")
                    assert mount.get('type')=='fixed'
                    assert mount.find('parent').get('link')==prefix+'base_link'
                    assert mount.find('child').get('link')==motor_name
                    assert all(abs(a-b)<1e-9 for a,b in zip(values(mount.find('origin'),'xyz'),[x,sign*.0925,.023]))
                    assert all(abs(a-b)<1e-9 for a,b in zip(values(mount.find('origin'),'rpy'),[0,0 if end=='front' else math.pi,0]))
                    if end=='front':
                        assert x==old_wheel_origin[0], 'Front motor X differs from stock Waffle'
                    # Full motor case stays within the plate fore/aft limits;
                    # lateral projection remains intentional (now ~19.1 mm).
                    case_x=(x-.03525,x+.01125) if end=='front' else (x-.01125,x+.03525)
                    assert case_x[0]>bounds[0][0]-.064 and case_x[1]<bounds[0][1]-.064, 'Case projects beyond plate front/rear'
                    motor=new.find(f"link[@name='{motor_name}']/visual/geometry/mesh")
                    assert motor.get('filename').endswith(f'/motor_{side}.stl')
                    low,high=motor_z_bounds[side]
                    bottom=.023+(low if end=='front' else -high)
                    assert abs(bottom-floor_top)<1e-7, 'Motor-to-mounting-plate gap/penetration'
                    motor_box=new.find(f"link[@name='{motor_name}']/collision/geometry/box")
                    assert abs(.023-float(motor_box.get('size').split()[2])/2-floor_top)<1e-7
                    rotor=new.find(f"joint[@name='{prefix}drive_{end}_{side}_joint']")
                    assert rotor.get('type')=='continuous'
                    assert rotor.find('parent').get('link')==motor_name
                    assert rotor.find('child').get('link')==rotor_name
                    assert rotor.find('axis').get('xyz')=='0 1 0'
                    assert rotor.find('origin').get('xyz')=='0 0 0', 'Non-coaxial motor and wheel'
                    assert rotor.find('mimic').attrib==dict(joint=prefix+f'wheel_{side}_joint',multiplier='1',offset='0')
                    hub=new.find(f"link[@name='{rotor_name}']/visual/geometry/cylinder")
                    assert hub is not None and float(hub.get('length'))==.012, 'Missing output hub'
            # Read actual source-mesh Y extents and URDF mounts, on BOTH axle
            # pairs. Rear pitch=pi does not change Y; do not confuse 95 mm
            # empty space with the 185 mm output-axis separation.
            for end in ('front','rear'):
                left_y=values(new.find(f"joint[@name='{prefix}motor_{end}_left_mount']/origin"),'xyz')[1]
                right_y=values(new.find(f"joint[@name='{prefix}motor_{end}_right_mount']/origin"),'xyz')[1]
                assert abs(left_y-right_y-.185)<1e-9, 'Left/right axle separation drift'
                facing_left=left_y+motor_y_bounds['left'][0]
                facing_right=right_y+motor_y_bounds['right'][1]
                assert abs(facing_left-.0475)<1e-7 and abs(facing_right+.0475)<1e-7
                assert abs(facing_left-facing_right-.095)<1e-7, 'Measured lateral 95 mm CASE gap drift'
                faces={}
                for side,y in (('left',left_y),('right',right_y)):
                    collision=new.find(f"link[@name='{prefix}motor_{end}_{side}_link']/collision")
                    cy=y+values(collision.find('origin'),'xyz')[1]
                    half=values(collision.find('geometry/box'),'size')[1]/2
                    faces[side]=cy-half if side=='left' else cy+half
                assert abs(faces['left']-faces['right']-.095)<1e-9, 'Collision/mesh case-gap mismatch'
            for mesh in new.findall('.//mesh'):
                uri = mesh.get('filename')
                path = PACKAGE/uri.split('/',3)[3] if uri.startswith('package://turtlebot3_manipulation_description/') else Path(uri.removeprefix('file://'))
                assert path.is_file(),path
                assert path.stat().st_size > 84,path
            for geo in new.findall('.//geometry'):
                for shape in geo:
                    for key in ('size','radius','length','scale'):
                        if shape.get(key):
                            assert all(float(v)>0 for v in shape.get(key).split())
            urdf = Path(temp)/'leader.urdf'
            urdf.write_text(xml)
            subprocess.run(['check_urdf',str(urdf)],check=True,stdout=subprocess.DEVNULL)
            urdf.write_text(check_rviz_compatibility(new))
            subprocess.run(['check_urdf',str(urdf)],check=True,stdout=subprocess.DEVNULL)
            print(f'PASS prefix={prefix!r} eef={camera} driver_tf={camera_tf}: {len(old_joints)-3} preserved joints; rear-half arm and measured stack')
    print('PASS: 8 variants; case gaps fore/aft 68 mm, lateral 95 mm; supports 40/55/55 mm; seated camera; RViz materials; hardware unchanged')


if __name__ == '__main__':
    main()
