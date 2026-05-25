# Map Editor

`grid_demo.html` is a self-contained HTML Canvas editor for the occupancy grid map.

Open it directly in a browser:

```text
map/grid_demo.html
```

It supports:

- importing a JSON map file;
- clicking cells to place or remove obstacles;
- placing start and goal poses on grid intersections;
- editing start and goal heading angles;
- exporting a JSON map file for the C++ planner.

The editor does not use `fetch()`, so it can run by double-clicking the HTML file.

`default_map.json` is a starter map using rectangular obstacles. Files exported
from the editor use single-cell obstacles:

```json
{
  "version": 1,
  "width": 60,
  "height": 36,
  "start": {"x": 6, "y": 6, "theta": 0},
  "goal": {"x": 52, "y": 28, "theta": 0},
  "obstacles": [
    {"x": 0, "y": 0},
    {"x": 1, "y": 0}
  ]
}
```

The C++ planner reads this JSON map, runs Hybrid A*, and writes
`output/result.json`. You can then open that file with `tool/view_path_json.sh`
or `tool/view_path_json.bat`.
