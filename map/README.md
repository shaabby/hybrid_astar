# Map Editor

`grid_demo.html` is a self-contained HTML Canvas editor for the occupancy grid map.

Open it directly in a browser:

```text
map/grid_demo.html
```

It supports:

- importing a JSON map file;
- clicking cells to place or remove obstacles;
- placing start and goal poses;
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
  "start": {"x": 6.5, "y": 6.5, "theta": 0},
  "goal": {"x": 52.5, "y": 28.5, "theta": 0},
  "obstacles": [
    {"x": 0, "y": 0},
    {"x": 1, "y": 0}
  ]
}
```

The later C++ `main` program should read this JSON map, run Hybrid A*, then
generate `output/demo.html` with the planned vehicle poses embedded inside.
