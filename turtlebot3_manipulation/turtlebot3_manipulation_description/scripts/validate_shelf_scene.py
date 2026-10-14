#!/usr/bin/env python3
# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Check labelled dimensions, open fronts and conservative collision envelopes."""

import xml.etree.ElementTree as ET

from export_shelf_scene import add_shelf, FLOOR_TOP, HEIGHT


def main():
    """Test both nominal and nested layouts without launching ROS."""
    for pitch in (.156, .168, .180):
        root = ET.Element('robot', name='test_shelf')
        ET.SubElement(root, 'link', name='world')
        add_shelf(root, 'world', pitch)
        names = [node.get('name') for node in root if node.tag in ('joint', 'link')]
        assert len(names) == len(set(names))
        for level in (1, 2):
            link = root.find(f"link[@name='shelf_level_{level}']")
            floor = link.find("visual[@name='floor']/geometry/box")
            assert list(map(float, floor.get('size').split())) == [.285, .365, .004]
            collisions = link.findall('collision')
            assert len(collisions) == 9  # Floor, feet(4), lip, sides(2), back.
            assert not any('front_wall' in c.get('name') for c in collisions)
            assert len(link.findall('visual')) > 60
            floor_joint = root.find(f"joint[@name='shelf_floor_{level}_fixed']/origin")
            assert abs(float(floor_joint.get('xyz').split()[2]) - FLOOR_TOP) < 1e-9
            # Primitive extents must not exceed the external label dimensions.
            for element in link.findall('visual') + collisions:
                size = list(map(float, element.find('geometry/box').get('size').split()))
                xyz = list(map(float, element.find('origin').get('xyz').split()))
                assert all(value > 0 for value in size)
                assert abs(xyz[0])+size[0]/2 <= .285/2+1e-9
                assert abs(xyz[1])+size[1]/2 <= .365/2+1e-9
                assert xyz[2]-size[2]/2 >= -1e-9
                assert xyz[2]+size[2]/2 <= HEIGHT+1e-9
        print(f'PASS: 2 bins, pitch={pitch:.3f}, total={HEIGHT+pitch:.3f} m')


if __name__ == '__main__':
    main()
