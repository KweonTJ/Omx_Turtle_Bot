# Copyright 2026 ktj
# Licensed under the Apache License, Version 2.0
"""Generate exact vector ArUco print artwork, not artistic marker images."""

import argparse
from pathlib import Path

import cv2


def marker_bitmap(dictionary, marker_id, pixels):
    """Generate the marker with the same one-bit border used by the detector."""
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
    return cv2.aruco.drawMarker(dictionary, marker_id, pixels)


def print_sheet():
    """Return an A4 SVG with four 40 mm black squares and a 100 mm ruler."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="210mm" height="297mm" '
        'viewBox="0 0 210 297">',
        '<rect width="210" height="297" fill="white"/>',
        '<g fill="black" font-family="sans-serif">',
        '<text x="15" y="20" font-size="6">OMX / DICT_4X4_50 / ID 1-4</text>',
        '<text x="15" y="29" font-size="4">Print 100% / black square = 40 mm</text>',
    ]
    for marker_id, (x, y) in enumerate(((30, 50), (130, 50),
                                       (30, 140), (130, 140)), start=1):
        pixels = marker_bitmap(dictionary, marker_id, 6)
        for row in range(6):
            for col in range(6):
                if pixels[row, col] == 0:
                    parts.append(
                        f'<rect x="{x+col*40/6:.8f}" y="{y+row*40/6:.8f}" '
                        f'width="{40/6:.8f}" height="{40/6:.8f}"/>')
        parts.append(f'<text x="{x}" y="{y+51}" font-size="5">ID {marker_id}</text>')
    parts.extend([
        '<path d="M30 235 H130 M30 232 V238 M130 232 V238" '
        'fill="none" stroke="black" stroke-width="0.3"/>',
        '<text x="30" y="247" font-size="4">100 mm ruler - measure after printing</text>',
        '<text x="15" y="268" font-size="4">Keep white margin. No fit-to-page.</text>',
        '<text x="15" y="277" font-size="4">Attach flat, facing the gripper camera.</text>',
        '</g></svg>',
    ])
    return '\n'.join(parts) + '\n'


def main():
    """Save the generated sheet to a user-specified new file."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New .svg output path')
    args = parser.parse_args()
    with args.output.open('x', encoding='utf-8') as stream:
        stream.write(print_sheet())
    print(f'Created {args.output}: A4, DICT_4X4_50, IDs 1-4, black square 40 mm')


if __name__ == '__main__':
    main()
