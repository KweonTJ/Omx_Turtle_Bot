#!/usr/bin/env python3
"""Generate metre-scale leader display meshes from existing ROBOTIS assets.

Requires numpy/trimesh. Extract the actual perforated plates from the repository's
ROBOTIS base STL and assemble THREE half plates on EVERY level, without stretching.
Two opposing half plates form the front octagon; the third meets its narrow rear
edge, with the final rear half absent. Extract four motor bodies and sprockets
from existing ROBOTIS geometry. Belt and shaft/bracket details are visual proxies.
"""
import argparse
import math
from pathlib import Path
import struct
import numpy as np
import trimesh


def box(size, center=(0, 0, 0)):
    vertices = [(center[0]+x*size[0]/2, center[1]+y*size[1]/2,
                 center[2]+z*size[2]/2)
                for x, y, z in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                                (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
    faces = [(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
             (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
    return [[vertices[i] for i in face] for face in faces]


def robotis_three_half_plates(parts):
    # Existing base.stl is in mm. Identify complete plate components by geometry,
    # not unstable split indices. Do not extract electronics or wheel supports.
    selected = [p for p in parts if len(p.faces) == 2582
                and np.isclose(p.extents[2],9,atol=.001)
                and np.isclose(p.bounds[0,2],82.5,atol=.001)
                and np.max(np.abs(p.bounds[:,1])) < 69]
    if len(selected) != 4:
        raise ValueError('ROBOTIS source mesh changed: review plate extraction')
    selected.sort(key=lambda p:p.bounds[:,0].mean())
    centers = [p.bounds[:,0].mean() for p in selected]
    if not np.allclose(np.diff(centers),64,atol=.002):
        raise ValueError('ROBOTIS source half-plate pitch changed')
    template = selected[0].copy()
    template.apply_translation(-template.bounds.mean(axis=0))
    assembly = []
    # Template's narrow outer face points -X. Opposing +/-90-degree halves
    # make a complete octagon at X=+32. Rear half points +X, meeting the
    # octagon's narrow -X edge. Its straight rear edge is the missing-half seam.
    for x,y,angle in ((32,-32,math.pi/2),(32,32,-math.pi/2),(-64,0,math.pi)):
        half = template.copy()
        half.apply_transform(trimesh.transformations.rotation_matrix(angle,[0,0,1]))
        half.apply_translation([x,y,0])
        assembly.append(half)
    result = trimesh.util.concatenate(assembly)
    assert len(result.faces) == 3*2582
    # Only unit conversion; preserve thickness, hole pitch and edge geometry.
    print('Each layer: 3 half plates, extents (mm):',result.extents.round(4))
    return (result.triangles*.001).tolist()


def motor(parts, side):
    # Source motor output axis is X=64, Z=23.25 mm, with the wheel centred at
    # Y=+/-144 mm. Preserve this motor-to-wheel mounting offset, not a box proxy.
    sign = 1 if side == 'left' else -1
    bodies = [p for p in parts if len(p.faces)==1220
              and np.allclose(p.extents,[46.5,40.4,28.5],atol=.002)
              and np.sign(p.bounds.mean(axis=0)[1])==sign]
    if len(bodies)!=1:
        raise ValueError('ROBOTIS source motor changed: review extraction')
    body=bodies[0]
    # Check the shaft centre from a cross-section, not the rounded legacy
    # wheel TF (Z=23). Otherwise rear pitch=pi gives a different mounting gap.
    shaft=body.section(plane_origin=[0,sign*139,0],plane_normal=[0,1,0])
    if shaft is None or len(shaft.discrete)!=1:
        raise ValueError('Source motor shaft section changed')
    curve=shaft.discrete[0]
    axis=(curve.min(axis=0)+curve.max(axis=0))/2
    if not np.allclose(axis[[0,2]],[64,23.25],atol=.002):
        raise ValueError('Source motor output axis changed')
    # Check the four actual holes on the plate-facing case surface (24 x 12 mm).
    section=body.section(plane_origin=[0,0,10],plane_normal=[0,0,1])
    holes=[]
    for curve in section.discrete:
        if np.max(np.ptp(curve[:,:2],axis=0))<3:
            holes.append((curve.min(axis=0)+curve.max(axis=0))[:2]/2)
    expected=sorted((x,sign*y) for x in (36,60) for y in (110,122))
    actual=sorted(tuple(v) for v in holes)
    if len(actual)!=4 or not np.allclose(actual,expected,atol=.002):
        raise ValueError('ROBOTIS four-hole mounting pattern changed')
    # Include the eight small case screws and connector contained by its bounds.
    contained=[p for p in parts if len(p.faces) in (16,32)
               and np.all(p.bounds[0]>=body.bounds[0]-.001)
               and np.all(p.bounds[1]<=body.bounds[1]+.001)]
    if len(contained)!=9:
        raise ValueError('ROBOTIS motor case details changed')
    result=trimesh.util.concatenate([body,*contained])
    result.apply_translation([-64,-sign*144,-23.25])
    print(f'{side} motor: 4 mounting holes, 24 x 12 mm; inner row 34 mm from axle')
    return (result.triangles*.001).tolist()


def track():
    # Match leader_drive_front/rear_x: measured 68 mm CASE GAP, not axle spacing.
    # A static belt belongs to base_link, never to a spinning wheel joint.
    faces = []
    front_x = 0.
    rear_x = front_x - .068 - 2*.03525  # 138.5 mm axle spacing from source case bounds
    radius, straight, center_x = .035, front_x-rear_x, (front_x+rear_x)/2
    perimeter = 2*straight+2*math.pi*radius
    # Retain the old ~12.24 mm visual pitch instead of shrinking 50 links.
    count = round(perimeter/.01224)  # 41 illustrative divisions, NOT measured count
    pitch = perimeter/count
    print(f'Belt: axle spacing {straight*1000:.1f} mm; {count} visual divisions; pitch {pitch*1000:.2f} mm')
    for i in range(count):
        s = (i+.5)*pitch
        if s < straight:
            x, z, angle = -straight/2+s, radius, 0
        elif s < straight+math.pi*radius:
            a = math.pi/2-(s-straight)/radius
            x, z, angle = straight/2+radius*math.cos(a), radius*math.sin(a), a-math.pi/2
        elif s < 2*straight+math.pi*radius:
            x, z, angle = straight/2-(s-straight-math.pi*radius), -radius, -math.pi
        else:
            a = -math.pi/2-(s-2*straight-math.pi*radius)/radius
            x, z, angle = -straight/2+radius*math.cos(a), radius*math.sin(a), a-math.pi/2
        # Open rectangular windows, side rails and centre rib like the observed tread.
        parts = []
        for edge in (-1, 1):
            parts += box((pitch*.88, .005, .006), (0, edge*.0185, 0))
            parts += box((.003, .042, .006), (edge*pitch*.33, 0, 0))
        parts += box((pitch*.88, .004, .008))
        c, sn = math.cos(angle), math.sin(angle)
        faces.extend([[(center_x+x+c*px-sn*pz, py, z+sn*px+c*pz)
                       for px, py, pz in triangle] for triangle in parts])
    return faces


def sprocket(source):
    parts = trimesh.load(source).split(only_watertight=False)
    selected = [p for p in parts if np.allclose(p.extents,[61.229,17.5,61.229],atol=.002)]
    if len(selected) != 1:
        raise ValueError('ROBOTIS source mesh changed: review sprocket extraction')
    return (selected[0].triangles*.001).tolist()


def write_stl(path, faces):
    with path.open('wb') as stream:
        stream.write(b'Video-derived leader visual proxy, metres; not manufacturer CAD'.ljust(80,b'\0'))
        stream.write(struct.pack('<I',len(faces)))
        for a,b,c in faces:
            u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
            normal = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            norm = math.sqrt(sum(x*x for x in normal)) or 1
            stream.write(struct.pack('<12fH',*[x/norm for x in normal],*a,*b,*c,0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,
                        default=Path(__file__).resolve().parents[1]/'meshes/leader_video')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    source = Path(__file__).resolve().parents[1]/'meshes/turtlebot3_waffle_pi/base.stl'
    parts = trimesh.load(source).split(only_watertight=False)
    plate = robotis_three_half_plates(parts)
    shapes = {
        'lower_plate.stl': plate,
        'camera_plate.stl': plate,
        'upper_plate.stl': plate,
        'track.stl': track(), 'sprocket.stl': sprocket(source.with_name('left_tire.stl')),
        'motor_left.stl': motor(parts,'left'),
        'motor_right.stl': motor(parts,'right'),
    }
    for name, faces in shapes.items():
        write_stl(args.output_dir/name,faces)
        print(f'{name}: {len(faces)} triangles')


if __name__ == '__main__':
    main()
