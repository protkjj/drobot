from __future__ import annotations
from pathlib import Path
import random
from datetime import datetime


def generate_sdf_world(num_obstacles: int, seed: int | None = None) -> str:
    rng = random.Random(seed)

    models = []
    for i in range(num_obstacles):
        x = rng.uniform(-4.0, 4.0)
        y = rng.uniform(-4.0, 4.0)
        sx = rng.uniform(0.2, 1.0)
        sy = rng.uniform(0.2, 1.0)
        sz = rng.uniform(0.2, 1.0)

        models.append(f"""
    <model name="box_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {sz/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>
          </geometry>
        </visual>
      </link>
    </model>
""")

    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="generated_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    {''.join(models)}
  </world>
</sdf>
"""


def write_world(out_path: Path, num_obstacles: int, seed: int | None = None) -> Path:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    sdf = generate_sdf_world(num_obstacles=num_obstacles, seed=seed)
    out_path.write_text(sdf)
    return out_path


def default_output_path(base_dir: Path) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return base_dir / f"generated_{ts}.sdf"
