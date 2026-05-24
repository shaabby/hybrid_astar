#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

BASE_WIDTH = 60
BASE_HEIGHT = 36


def parse_size(text):
    if "x" not in text:
        raise argparse.ArgumentTypeError(f"size must be WIDTHxHEIGHT: {text}")
    width_text, height_text = text.lower().split("x", 1)
    try:
        width = int(width_text)
        height = int(height_text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid size: {text}") from error
    if width < 20 or height < 16:
        raise argparse.ArgumentTypeError("size is too small; minimum is 20x16")
    return width, height


def scaled(value, source, target):
    return max(1, int(round(value * target / source)))


def sx(value, width):
    return scaled(value, BASE_WIDTH, width)


def sy(value, height):
    return scaled(value, BASE_HEIGHT, height)


def pose(x, y, theta, width, height):
    return {
        "x": min(width - 2, max(1, sx(x, width))),
        "y": min(height - 2, max(1, sy(y, height))),
        "theta": theta,
    }


def add_cell(obstacles, width, height, x, y):
    if 0 <= x < width and 0 <= y < height:
        obstacles.add((x, y))


def add_rect(obstacles, width, height, x, y, w, h):
    x0 = min(width - 1, max(0, sx(x, width)))
    y0 = min(height - 1, max(0, sy(y, height)))
    rw = max(1, sx(w, width))
    rh = max(1, sy(h, height))
    for yy in range(y0, min(height, y0 + rh)):
        for xx in range(x0, min(width, x0 + rw)):
            add_cell(obstacles, width, height, xx, yy)


def add_border(obstacles, width, height):
    for x in range(width):
        obstacles.add((x, 0))
        obstacles.add((x, height - 1))
    for y in range(height):
        obstacles.add((0, y))
        obstacles.add((width - 1, y))


def clear_rect(obstacles, width, height, x, y, w, h):
    x0 = min(width - 1, max(0, sx(x, width)))
    y0 = min(height - 1, max(0, sy(y, height)))
    rw = max(1, sx(w, width))
    rh = max(1, sy(h, height))
    for yy in range(y0, min(height, y0 + rh)):
        for xx in range(x0, min(width, x0 + rw)):
            obstacles.discard((xx, yy))


def clear_pose_area(obstacles, width, height, p, radius=2):
    cx = int(round(p["x"]))
    cy = int(round(p["y"]))
    for yy in range(cy - radius, cy + radius + 1):
        for xx in range(cx - radius, cx + radius + 1):
            if 0 < xx < width - 1 and 0 < yy < height - 1:
                obstacles.discard((xx, yy))


def make_map(width, height, start, goal, obstacles):
    clear_pose_area(obstacles, width, height, start)
    clear_pose_area(obstacles, width, height, goal)
    return {
        "version": 1,
        "width": width,
        "height": height,
        "start": start,
        "goal": goal,
        "obstacles": [
            {"x": x, "y": y}
            for x, y in sorted(obstacles, key=lambda cell: (cell[1], cell[0]))
        ],
    }


def write_map(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def empty_maps(width, height):
    maps = []
    for index, (start_base, goal_base) in enumerate([
        ((6, 6, 0.0), (52, 28, 0.0)),
        ((8, 28, -0.35), (52, 8, 0.35)),
    ], start=1):
        obstacles = set()
        add_border(obstacles, width, height)
        start = pose(*start_base, width, height)
        goal = pose(*goal_base, width, height)
        maps.append((f"empty{index:02d}", make_map(width, height, start, goal, obstacles)))
    return maps


def simple_maps(width, height):
    specs = [
        (
            "simple01",
            (6, 6, 0.0),
            (52, 28, 0.0),
            [(18, 8, 4, 16), (34, 12, 4, 16)],
        ),
        (
            "simple02",
            (8, 28, -0.25),
            (52, 7, 0.25),
            [(14, 15, 16, 3), (36, 18, 14, 3), (28, 7, 3, 8)],
        ),
        (
            "simple03",
            (6, 18, 0.0),
            (53, 18, 0.0),
            [(18, 6, 3, 14), (30, 16, 3, 14), (42, 6, 3, 14)],
        ),
    ]
    return maps_from_specs(width, height, specs)


def narrow_maps(width, height):
    specs = [
        (
            "narrow01",
            (6, 6, 0.0),
            (52, 29, 0.0),
            [(10, 10, 38, 3), (12, 23, 38, 3), (47, 12, 3, 11)],
        ),
        (
            "narrow02",
            (6, 29, 0.0),
            (53, 6, 0.0),
            [(12, 5, 3, 23), (24, 8, 3, 23), (36, 5, 3, 23), (48, 8, 3, 20)],
        ),
        (
            "narrow03",
            (7, 18, 0.0),
            (53, 18, 0.0),
            [(14, 8, 34, 3), (14, 25, 34, 3), (27, 11, 3, 14), (39, 11, 3, 14)],
        ),
    ]
    return maps_from_specs(width, height, specs)


def reverse_maps(width, height):
    specs = [
        (
            "reverse01",
            (16, 18, 0.0),
            (10, 18, 3.141593),
            [(5, 10, 22, 3), (5, 23, 22, 3), (27, 10, 3, 16)],
        ),
        (
            "reverse02",
            (44, 18, 3.141593),
            (51, 18, 0.0),
            [(33, 10, 22, 3), (33, 23, 22, 3), (30, 10, 3, 16)],
        ),
    ]
    return maps_from_specs(width, height, specs)


def u_maps(width, height):
    specs = [
        (
            "u01",
            (11, 18, 0.0),
            (49, 18, 3.141593),
            [(18, 8, 4, 20), (18, 8, 24, 4), (18, 24, 24, 4), (42, 8, 4, 20)],
        ),
        (
            "u02",
            (49, 18, 3.141593),
            (11, 18, 0.0),
            [(18, 7, 4, 22), (18, 25, 26, 4), (44, 7, 4, 22)],
        ),
    ]
    return maps_from_specs(width, height, specs)


def unreach_maps(width, height):
    specs = [
        (
            "unreach01",
            (6, 6, 0.0),
            (52, 28, 0.0),
            [(45, 21, 14, 14)],
        ),
    ]
    return maps_from_specs(width, height, specs)


def maps_from_specs(width, height, specs):
    maps = []
    for name, start_base, goal_base, rects in specs:
        obstacles = set()
        add_border(obstacles, width, height)
        for rect in rects:
            add_rect(obstacles, width, height, *rect)
        start = pose(*start_base, width, height)
        goal = pose(*goal_base, width, height)
        maps.append((name, make_map(width, height, start, goal, obstacles)))
    return maps


def build_maps(width, height):
    maps = []
    maps.extend(empty_maps(width, height))
    maps.extend(simple_maps(width, height))
    maps.extend(narrow_maps(width, height))
    maps.extend(reverse_maps(width, height))
    maps.extend(u_maps(width, height))
    maps.extend(unreach_maps(width, height))
    return maps


def parse_args():
    parser = argparse.ArgumentParser(description="Generate Hybrid A* testbench maps")
    parser.add_argument("--output-dir", default="map/generated")
    parser.add_argument("--width", type=int, default=BASE_WIDTH)
    parser.add_argument("--height", type=int, default=BASE_HEIGHT)
    parser.add_argument("--sizes", type=parse_size, nargs="*")
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output_dir)
    sizes = args.sizes if args.sizes else [(args.width, args.height)]

    generated = []
    for width, height in sizes:
        prefix = "" if len(sizes) == 1 else f"{width}x{height}_"
        for name, data in build_maps(width, height):
            path = output_dir / f"{prefix}{name}.json"
            write_map(path, data)
            generated.append(path)

    index_path = output_dir / "maps.txt"
    index_path.parent.mkdir(parents=True, exist_ok=True)
    index_path.write_text(
        "\n".join(path.as_posix() for path in generated) + "\n",
        encoding="utf-8",
    )

    print(f"generated {len(generated)} maps")
    print(f"output: {output_dir}")
    print(f"index: {index_path}")


if __name__ == "__main__":
    main()
