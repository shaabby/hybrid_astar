#include "HtmlWriter.hpp"

std::string HtmlWriter::wrap(const std::string& json) {
    return R"(<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Hybrid A* Demo</title>
  <style>
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background: #f4f5f7;
      color: #17202a;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main { width: min(1120px, calc(100vw - 32px)); }
    .bar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 12px;
    }
    h1 { margin: 0; font-size: 20px; }
    button {
      border: 1px solid #b8c0cc;
      background: white;
      border-radius: 6px;
      padding: 8px 12px;
      font: inherit;
      cursor: pointer;
    }
    canvas {
      display: block;
      width: 100%;
      height: auto;
      background: white;
      border: 1px solid #cfd6df;
    }
  </style>
</head>
<body>
  <main>
    <div class="bar">
      <h1>Hybrid A* Output Loop Demo: Straight Vehicle Motion</h1>
      <button id="toggle" type="button">Pause</button>
    </div>
    <canvas id="canvas" width="1100" height="720"></canvas>
  </main>

  <script id="planner-data" type="application/json">
)" + json + R"(  </script>

  <script>
    const data = JSON.parse(document.getElementById("planner-data").textContent);
    const canvas = document.getElementById("canvas");
    const ctx = canvas.getContext("2d");
    const margin = 42;
    const scale = Math.min(
      (canvas.width - margin * 2) / data.map.width,
      (canvas.height - margin * 2) / data.map.height
    );
    let frame = 0;
    let playing = true;

    function wx(x) { return margin + x * scale; }
    function wy(y) { return margin + (data.map.height - y) * scale; }

    function drawGrid() {
      ctx.strokeStyle = "#e5e7eb";
      ctx.lineWidth = 1;
      for (let x = 0; x <= data.map.width; ++x) {
        ctx.beginPath();
        ctx.moveTo(wx(x), wy(0));
        ctx.lineTo(wx(x), wy(data.map.height));
        ctx.stroke();
      }
      for (let y = 0; y <= data.map.height; ++y) {
        ctx.beginPath();
        ctx.moveTo(wx(0), wy(y));
        ctx.lineTo(wx(data.map.width), wy(y));
        ctx.stroke();
      }
    }

    function drawObstacles() {
      ctx.fillStyle = "#111827";
      for (const obs of data.map.obstacles) {
        ctx.fillRect(wx(obs.x), wy(obs.y + 1), scale, scale);
      }
    }

    function drawPose(pose, color, label) {
      const x = wx(pose.x);
      const y = wy(pose.y);
      ctx.save();
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(x, y, scale * 0.36, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = color;
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(x + Math.cos(pose.theta) * scale * 1.2, y - Math.sin(pose.theta) * scale * 1.2);
      ctx.stroke();
      ctx.fillStyle = "#111827";
      ctx.font = "700 14px system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(label, x, y - scale * 0.72);
      ctx.restore();
    }

    function drawPath(limit) {
      ctx.strokeStyle = "#2563eb";
      ctx.lineWidth = 3;
      ctx.beginPath();
      for (let i = 0; i <= limit; ++i) {
        const p = data.path[i];
        if (i === 0) ctx.moveTo(wx(p.x), wy(p.y));
        else ctx.lineTo(wx(p.x), wy(p.y));
      }
      ctx.stroke();
    }

    function drawCar(pose) {
      const v = data.vehicle;
      ctx.save();
      ctx.translate(wx(pose.x), wy(pose.y));
      ctx.rotate(-pose.theta);

      ctx.fillStyle = "#f97316";
      ctx.strokeStyle = "#9a3412";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.rect(
        -v.rearToCenter * scale,
        -v.width * scale / 2,
        v.length * scale,
        v.width * scale
      );
      ctx.fill();
      ctx.stroke();

      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo((v.length - v.rearToCenter) * scale, 0);
      ctx.stroke();

      ctx.fillStyle = "#ffffff";
      ctx.beginPath();
      ctx.arc(0, 0, 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }

    function draw() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      drawGrid();
      drawObstacles();
      drawPose(data.map.start, "#16a34a", "S");
      drawPose(data.map.goal, "#dc2626", "G");
      drawPath(frame);
      drawCar(data.path[frame]);
    }

    function tick() {
      if (playing) {
        frame = (frame + 1) % data.path.length;
      }
      draw();
      requestAnimationFrame(tick);
    }

    document.getElementById("toggle").addEventListener("click", () => {
      playing = !playing;
      document.getElementById("toggle").textContent = playing ? "Pause" : "Play";
    });

    draw();
    requestAnimationFrame(tick);
  </script>
</body>
</html>
)";
}