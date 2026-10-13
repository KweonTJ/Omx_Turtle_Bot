#!/usr/bin/env python3
"""Generate original illustrative PCB silkscreen textures (not schematics/CAD).

Only UV-mapped box visuals use these textures. Original ROBOTIS STLs are not
modified or given fictitious UV coordinates. Requires Pillow; no downloads.
"""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont


def pcb(path, label, background, copper):
    image = Image.new('RGB', (1024, 768), background)
    draw = ImageDraw.Draw(image)
    font_path = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf')
    font = ImageFont.truetype(str(font_path), 48) if font_path.exists() else ImageFont.load_default()
    small = ImageFont.truetype(str(font_path), 22) if font_path.exists() else ImageFont.load_default()
    # Decorative traces/pads. Intentionally not a real electrical netlist.
    for i in range(15):
        x = 45 + i * 65
        y = 90 + (i % 4) * 28
        draw.line([(x, 50), (x, y), (x + 22, y + 22), (x + 22, 550)], fill=copper, width=3)
        for py in (50, 550):
            draw.ellipse((x-5, py-5, x+5, py+5), fill='#d4b958')
    for y in (60, 700):
        for x in range(110, 940, 24):
            draw.rectangle((x-4, y-7, x+4, y+7), fill='#d5c58b')
    for x in (54, 970):
        for y in (54, 714):
            draw.ellipse((x-20,y-20,x+20,y+20), fill='#d6b860')
            draw.ellipse((x-10,y-10,x+10,y+10), fill='#192b27')
    draw.rounded_rectangle((65, 572, 960, 672), radius=8, fill=background, outline='#d4e7df', width=3)
    draw.text((92, 586), label, fill='#eef4df', font=font)
    draw.text((90, 640), 'VISUAL TWIN / ILLUSTRATIVE LAYOUT', fill='#e0e8d4', font=small)
    image.save(path)


def main():
    directory = Path(__file__).resolve().parents[1] / 'meshes/leader_video'
    directory.mkdir(parents=True, exist_ok=True)
    pcb(directory/'opencr_pcb.png', 'OpenCR 1.0', '#16723c', '#3f9555')
    pcb(directory/'jetson_pcb.png', 'JETSON ORIN NANO', '#096950', '#368c70')
    print(f'Generated two original PCB textures in {directory}')


if __name__ == '__main__':
    main()
