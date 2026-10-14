#!/usr/bin/env python3
# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Render two URDF-based shelf inspection views without running robot nodes."""

import argparse
from pathlib import Path

from render_leader_preview import build_scene
import mujoco
from PIL import Image, ImageDraw


def main():
    """Use the same URDF geometry as RViz, not a hand-made scene substitute."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('--package', type=Path,
                        default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--with-leader', action='store_true')
    args = parser.parse_args()
    poses = {'joint2': -1.15, 'joint3': .60, 'joint4': .55,
             'gripper_left_joint': .010}
    model = mujoco.MjModel.from_xml_string(
        build_scene(args.urdf, args.package, poses))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    sheet = Image.new('RGB', (1600, 890), '#e5e9ee')
    draw = ImageDraw.Draw(sheet)
    draw.text((25, 18), 'DAISO 1001965 | TWO OPEN-FRONT BINS | DISPLAY APPROXIMATION',
              fill='#172b40')
    draw.text((25, 42), 'Each: W365 x D285 x H180 mm | Inner: W332 x D245 x H152 mm',
              fill='#42566b')
    draw.text((25, 63), 'Nesting, wall/feet/lip shape and robot placement require measurement.',
              fill='#42566b')
    with mujoco.Renderer(model, height=800, width=800) as renderer:
        for index, (azimuth, elevation, label) in enumerate([
                (35, -22, 'OPEN FRONT / LEFT'), (325, -15, 'OPEN FRONT / RIGHT')]):
            camera = mujoco.MjvCamera()
            camera.lookat[:] = [.20 if args.with_leader else 0, 0, .17]
            camera.distance = 1.30 if args.with_leader else .85
            camera.azimuth, camera.elevation = azimuth, elevation
            renderer.update_scene(data, camera=camera)
            tile = Image.fromarray(renderer.render())
            ImageDraw.Draw(tile).text((18, 18), label, fill='#172b40')
            sheet.paste(tile, (800*index, 90))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.output)
    print(args.output)


if __name__ == '__main__':
    main()
