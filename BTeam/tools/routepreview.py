"""
ROUTE PREVIEW
Draws an auton from src/auton/auton.cc on the field, the way Jerry.io shows it,
so you can check points and lines without running the robot.

It reads the moves straight out of the auton's code (the numbers have to be
written in the move, like pointTo(120, -150, ...)), then plays them back
perfectly: straight lines, exact turns, no sliding. So it shows what the code
ASKS for, not what the robot really does.

Usage (from the BTeam folder):
  python tools/routepreview.py                 match auton, left side
  python tools/routepreview.py --side right    flipped for the right side
  python tools/routepreview.py --auton skills
  python tools/routepreview.py --field override   the VEX Override field
  python tools/routepreview.py --field my.png      any Jerry.io-sized image
  python tools/routepreview.py --seen "121,-150 118,-62"
      also draws where the robot really went (from a video), in pink
Saves tools/route_<auton>_<side>.png and prints every stop.
Field images from Jerry.io are in tools/fields (empty is the default).

Field: Jerry.io "VEX Gaming Positioning System", centimeters, (0, 0) at the
center. X+ toward our wall, Y+ to our right, heading 0 = +Y, clockwise.
Jerry.io field images cover 369 cm across, perimeter included.
"""

import argparse
import math
import re
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

# SETTINGS
START = (150.0, -150.0, 270.0)  # left side start: x, y, heading
ROBOT_LENGTH = 38.0  # cm, front to back
ROBOT_WIDTH = 38.0  # cm, side to side
FIELD_INSIDE = 140.4 * 2.54  # wall to wall, matches autonconfig.hpp
IMAGE_SPAN = 369.0  # cm across a Jerry.io field image
SIZE = 1000  # output image pixels

HERE = Path(__file__).resolve().parent
FIELDS = {"empty": HERE / "fields" / "empty.png",
          "override": HERE / "fields" / "vex-override.png"}
AUTON_FILE = HERE.parent / "src" / "auton" / "auton.cc"

NUMBER = r"(-?\d+(?:\.\d+)?)"


def auton_body(name):
    """The code inside void <name>(Settings::Side side) { ... }."""
    text = AUTON_FILE.read_text(encoding="utf-8")
    start = text.index("void %s(" % name)
    start = text.index("{", start) + 1
    depth = 1
    for i in range(start, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[start:i]
    raise ValueError("couldn't find the end of " + name)


def read_moves(body):
    """Each move in order, as (kind, numbers, backwards, line of code)."""
    moves = []
    pattern = re.compile(r"\b(pointTo|moveTo|turnTo|straightToY|driveStraight)\s*\(([^;]*)\);")
    for line in body.splitlines():
        code = line.split("//")[0]
        for match in pattern.finditer(code):
            kind, args = match.group(1), match.group(2)
            numbers = [float(n) for n in re.findall(NUMBER, args.split("{")[0])]
            backwards = "BACK" in args or "forwards = false" in args
            moves.append((kind, numbers, backwards, line.strip()))
    return moves


def facing(x, y, tx, ty, backwards):
    heading = math.degrees(math.atan2(tx - x, ty - y)) % 360
    return (heading + 180) % 360 if backwards else heading


def play(moves, right):
    """Every stop as (x, y, heading, description), on the chosen side."""
    flip = -1 if right else 1

    def side_heading(h):
        return (540 - h) % 360 if right else h

    x, y, heading = START[0], START[1] * flip, side_heading(START[2])
    stops = [(x, y, heading, "start")]
    for kind, n, backwards, line in moves:
        if kind == "pointTo":
            tx, ty = n[0], n[1] * flip
            if math.hypot(tx - x, ty - y) > 0.01:
                heading = facing(x, y, tx, ty, backwards)
            x, y = tx, ty
        elif kind == "moveTo":
            x, y, heading = n[0], n[1] * flip, side_heading(n[2])
        elif kind == "turnTo":
            heading = side_heading(n[0])
        elif kind == "straightToY":
            along = math.cos(math.radians(heading))
            if abs(along) >= 0.1:
                distance = (n[0] * flip - y) / along
                x += distance * math.sin(math.radians(heading))
                y = n[0] * flip
        elif kind == "driveStraight":
            x += n[0] * math.sin(math.radians(heading))
            y += n[0] * math.cos(math.radians(heading))
        stops.append((x, y, heading, line))
    return stops


def draw(stops, field_image, out_path, seen=()):
    scale = SIZE / IMAGE_SPAN

    def px(x, y):
        return (SIZE / 2 + x * scale, SIZE / 2 - y * scale)

    field_image = FIELDS.get(field_image, field_image)
    if field_image and Path(field_image).exists():
        image = Image.open(field_image).convert("RGB").resize((SIZE, SIZE))
    else:
        image = Image.new("RGB", (SIZE, SIZE), (60, 60, 60))
        grid = ImageDraw.Draw(image)
        half = FIELD_INSIDE / 2
        tile = FIELD_INSIDE / 6
        for i in range(7):  # 6 x 6 tiles
            v = -half + i * tile
            grid.line([px(v, -half), px(v, half)], fill=(90, 90, 90), width=1)
            grid.line([px(-half, v), px(half, v)], fill=(90, 90, 90), width=1)
        grid.rectangle([px(-half, half), px(half, -half)], outline=(230, 230, 230), width=4)
        grid.text(px(half - 40, 8), "OUR WALL", fill=(230, 230, 230))
    pen = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    # Coordinate marks every 50 cm, to read positions off the picture
    for v in range(-150, 151, 50):
        gx, _ = px(v, 0)
        _, gy = px(0, v)
        pen.line([(gx, 0), (gx, 8)], fill=(255, 255, 255), width=2)
        pen.text((gx + 3, 10), "x%d" % v, fill=(255, 255, 255), font=font)
        pen.line([(0, gy), (8, gy)], fill=(255, 255, 255), width=2)
        pen.text((10, gy - 6), "y%d" % v, fill=(255, 255, 255), font=font)

    for (x0, y0, _, _), (x1, y1, _, _) in zip(stops, stops[1:]):
        pen.line([px(x0, y0), px(x1, y1)], fill=(255, 215, 0), width=3)

    for i, (x, y, heading, _) in enumerate(stops):
        rad = math.radians(heading)
        fx, fy = math.sin(rad), math.cos(rad)  # forward
        rx, ry = math.cos(rad), -math.sin(rad)  # right
        hl, hw = ROBOT_LENGTH / 2, ROBOT_WIDTH / 2
        corners = [
            (x + fx * a + rx * b, y + fy * a + ry * b)
            for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))
        ]
        pen.polygon([px(*c) for c in corners], outline=(0, 200, 255))
        pen.line([px(x, y), px(x + fx * hl, y + fy * hl)], fill=(255, 60, 60), width=3)
        cx, cy = px(x, y)
        pen.text((cx + 4, cy + 4), str(i), fill=(255, 255, 255), font=font)

    # Where the robot really went
    seen_px = [px(x, y) for x, y in seen]
    if len(seen_px) > 1:
        pen.line(seen_px, fill=(255, 80, 200), width=3)
    for i, (cx, cy) in enumerate(seen_px):
        pen.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], outline=(255, 80, 200), width=2)
        pen.text((cx + 6, cy - 14), "s%d" % i, fill=(255, 80, 200), font=font)

    image.save(out_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--auton", default="match")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--field", default="empty",
                        help="empty, override, or a Jerry.io-sized image (369 cm across)")
    parser.add_argument("--seen", default="",
                        help='real positions, like "121,-150 118,-62"')
    args = parser.parse_args()

    stops = play(read_moves(auton_body(args.auton)), args.side == "right")
    for i, (x, y, heading, line) in enumerate(stops):
        print("%2d  x %7.1f  y %7.1f  heading %5.1f   %s" % (i, x, y, heading, line))
    out = HERE / ("route_%s_%s.png" % (args.auton, args.side))
    seen = [tuple(float(v) for v in pair.split(",")) for pair in args.seen.split()]
    draw(stops, args.field, out, seen)
    print("saved", out)


if __name__ == "__main__":
    main()
