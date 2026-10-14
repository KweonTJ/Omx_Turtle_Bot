#!/usr/bin/env python3
# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Two Daiso bins on a 400 x 400 mm support with its top 100 mm above ground.

Keep the old floor-standing shelf export unchanged. The unmeasured underside
of the support is conservatively represented as a solid collision envelope.
"""
import argparse
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from export_shelf_scene import add_shelf, box, fixed


def add_raised_shelf(root, parent, xy=(.300, 0.), height=.100,
                     size=(.400, .400), stack_pitch=.156):
    if not all(math.isfinite(v) for v in (*xy, height, *size, stack_pitch)):
        raise ValueError('Non-finite support dimensions')
    if height <= 0 or size[0] < .285 or size[1] < .365:
        raise ValueError('Support must be positive and contain the shelf footprint')
    support = fixed(root, parent, 'shelf_support', (*xy, 0))
    for collision in (False, True):
        box(support, 'support_envelope', (*size, height), (0, 0, height/2),
            collision=collision, color='.48 .32 .19 1')
    fixed(root, 'shelf_support', 'shelf_support_top', (0, 0, height))
    add_shelf(root, 'shelf_support_top', stack_pitch=stack_pitch)
    return root


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--with-leader', action='store_true')
    parser.add_argument('--shelf-x', type=float, default=.300)
    parser.add_argument('--shelf-y', type=float, default=0.)
    parser.add_argument('--support-height', type=float, default=.100)
    parser.add_argument('--stack-pitch', type=float, default=.156)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.with_leader:
        root = ET.fromstring(subprocess.check_output([
            '/usr/bin/python3', str(package/'scripts/export_leader_urdf.py'), '--stdout'], text=True))
        parent, xy = 'base_footprint', (args.shelf_x, args.shelf_y)
    else:
        root = ET.Element('robot')
        parent, xy = 'shelf_world', (0., 0.)
        ET.SubElement(root, 'link', name=parent)
    root.set('name', 'leader_raised_shelf' if args.with_leader else 'raised_two_level_shelf')
    add_raised_shelf(root, parent, xy, args.support_height, stack_pitch=args.stack_pitch)
    ET.indent(root, space='  ')
    text = '<?xml version="1.0"?>\n'+ET.tostring(root, encoding='unicode')+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding='utf-8')
        print(args.output)
    else:
        print(text, end='')


if __name__ == '__main__':
    main()
