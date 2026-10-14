#!/usr/bin/env python3
# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Generate the two open-front Daiso 1001965 bins as a display URDF.

Label dimensions are known. Shell thickness, feet, lip and stack interlock are
approximations. Collision envelopes are deliberately solid across the slots.
This is not a calibrated planning scene or a validated dynamics model.
Official product: https://www.daisomall.co.kr/pd/pdr/SCR_PDR_0001?pdNo=1001965
External size 365x285x180 mm and PP confirmed 2026-10-14. Stacking pitch is
not specified there; default 156 mm = label inner height + estimated 4 mm floor.
"""

import argparse
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


WIDTH, DEPTH, HEIGHT = .365, .285, .180
INNER_WIDTH, INNER_DEPTH, INNER_HEIGHT = .332, .245, .152
FLOOR_TOP, FLOOR_THICKNESS = HEIGHT - INNER_HEIGHT, .004
PLASTIC = '.73 .76 .79 1'


def box(link, name, size, xyz, collision=False, color=PLASTIC):
    """Add a visual or a conservative collision primitive to one link."""
    element = ET.SubElement(link, 'collision' if collision else 'visual', name=name)
    ET.SubElement(element, 'origin', xyz=' '.join(map(str, xyz)), rpy='0 0 0')
    geometry = ET.SubElement(element, 'geometry')
    ET.SubElement(geometry, 'box', size=' '.join(map(str, size)))
    if not collision:
        material = ET.SubElement(element, 'material', name='shelf_plastic')
        ET.SubElement(material, 'color', rgba=color)


def fixed(root, parent, child, xyz=(0, 0, 0), yaw=0):
    """Attach a uniquely named display link."""
    link = ET.SubElement(root, 'link', name=child)
    joint = ET.SubElement(root, 'joint', name=child+'_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'origin', xyz=' '.join(map(str, xyz)), rpy=f'0 0 {yaw}')
    return link


def slotted_panel(link, prefix, length, centre, along_x):
    """Approximate slotted plastic without overlapping coplanar visual faces."""
    bottom, top = FLOOR_TOP, HEIGHT
    rails = [bottom+.004 + index*(top-bottom-.008)/4 for index in range(5)]
    for index, z in enumerate(rails):
        size = (length, .006, .008) if along_x else (.006, length, .008)
        box(link, f'{prefix}_rail_{index}', size, (*centre, z))
    count = max(3, round(length/.015))
    for index in range(count):
        offset = -length/2+.004 + index*(length-.008)/(count-1)
        x, y = centre
        x, y = (x+offset, y) if along_x else (x, y+offset)
        for row in range(4):
            low, high = rails[row]+.004, rails[row+1]-.004
            size = (.004, .006, high-low) if along_x else (.006, .004, high-low)
            box(link, f'{prefix}_rib_{index}_{row}', size, (x, y, (low+high)/2))


def add_shelf(root, parent, stack_pitch=.156, xyz=(0, 0, 0), yaw=0):
    """Add two bins, floor reference frames, and open front (-X direction)."""
    if not math.isfinite(stack_pitch) or not .156 <= stack_pitch <= .18:
        raise ValueError('stack_pitch must be 0.156..0.180 m; measure actual nesting')
    if not all(math.isfinite(v) for v in (*xyz, yaw)):
        raise ValueError('Shelf placement must be finite')
    fixed(root, parent, 'shelf_origin', xyz, yaw)
    for level in (1, 2):
        name = f'shelf_level_{level}'
        link = fixed(root, 'shelf_origin', name, (0, 0, (level-1)*stack_pitch))
        floor = (WIDTH, DEPTH, FLOOR_THICKNESS)
        # Size order in URDF is X(depth), Y(width), Z(height).
        floor = (floor[1], floor[0], floor[2])
        floor_centre = (0, 0, FLOOR_TOP-FLOOR_THICKNESS/2)
        for collision in (False, True):
            box(link, 'floor', floor, floor_centre, collision)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    box(link, f'foot_{sx}_{sy}', (.020, .016, .024),
                        (sx*(DEPTH/2-.010), sy*(WIDTH/2-.008), .012), collision)
            # Front is open above this small estimated lip, not a drawer.
            box(link, 'front_lip', (.020, WIDTH, .006),
                (-DEPTH/2+.010, 0, FLOOR_TOP+.003), collision)
        for side in (-1, 1):
            slotted_panel(link, f'side_{side}', DEPTH,
                          (0, side*(WIDTH/2-.003)), True)
            box(link, f'side_envelope_{side}',
                (DEPTH, (WIDTH-INNER_WIDTH)/2, INNER_HEIGHT),
                (0, side*(WIDTH+INNER_WIDTH)/4,
                 FLOOR_TOP+INNER_HEIGHT/2), True)
        slotted_panel(link, 'back', WIDTH, (DEPTH/2-.003, 0), False)
        box(link, 'back_envelope', ((DEPTH-INNER_DEPTH)/2, INNER_WIDTH, INNER_HEIGHT),
            ((DEPTH+INNER_DEPTH)/4, 0, FLOOR_TOP+INNER_HEIGHT/2), True)
        fixed(root, name, f'shelf_floor_{level}', (0, 0, FLOOR_TOP))
    return root


def main():
    """Export standalone bins or bins next to the existing leader display model."""
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--with-leader', action='store_true')
    parser.add_argument('--stack-pitch', type=float, default=.156)
    parser.add_argument('--shelf-x', type=float, default=.450)
    parser.add_argument('--shelf-y', type=float, default=0.)
    parser.add_argument('--shelf-z', type=float, default=0.)
    parser.add_argument('--shelf-yaw', type=float, default=0.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.with_leader:
        xml = subprocess.check_output([
            '/usr/bin/python3', str(package/'scripts/export_leader_urdf.py'),
            '--stdout'], text=True)
        root = ET.fromstring(xml)
        root.set('name', 'leader_shelf_display')
        parent = 'base_footprint'
        xyz = (args.shelf_x, args.shelf_y, args.shelf_z)
    else:
        root = ET.Element('robot', name='daiso_two_level_shelf')
        parent = 'shelf_world'
        ET.SubElement(root, 'link', name=parent)
        xyz = (0, 0, 0)
    add_shelf(root, parent, args.stack_pitch, xyz, args.shelf_yaw)
    ET.indent(root, space='  ')
    xml = ('<?xml version="1.0"?>\n'
           '<!-- Display approximation: no measured nesting, mass or reachability. -->\n'
           + ET.tostring(root, encoding='unicode') + '\n')
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(xml, encoding='utf-8')
    else:
        print(xml, end='')


if __name__ == '__main__':
    main()
