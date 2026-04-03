#!/usr/bin/env python3
"""
Render a ROS2 Nav2 OccupancyGrid saved as <root>.pgm + <root>.yaml
to a PNG with labeled metric axes.

Usage:
  python map_root_to_png.py my_map        # reads my_map.yaml + (its PGM) -> my_map.png
  python map_root_to_png.py my_map --dpi 300
"""

from pathlib import Path
import argparse
import yaml
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def load_yaml(yaml_path: Path) -> dict:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    return data

def load_pgm(pgm_path: Path) -> np.ndarray:
    img = Image.open(pgm_path)
    arr = np.array(img)
    # Normalize to uint8 for consistent plotting
    if arr.dtype == np.uint8:
        return arr
    arr = arr.astype(np.float64)
    maxv = arr.max() if arr.max() > 0 else 1.0
    arr = (arr / maxv * 255.0).round().astype(np.uint8)
    return arr

def main():
    ap = argparse.ArgumentParser(description="Render <root>.pgm + <root>.yaml to <root>.png with metric axes.")
    ap.add_argument("root", help="Map root name (without extension), e.g., 'my_map'")
    ap.add_argument("--dpi", type=int, default=300, help="Output PNG DPI (default: 300)")
    ap.add_argument("--grid", action="store_true", help="Draw a light grid on the axes")
    args = ap.parse_args()

    root = Path(args.root).with_suffix("")  # strip any accidental extension
    yaml_path = root.with_suffix(".yaml")
    if not yaml_path.exists():
        yaml_path = root.with_suffix(".yml")
        if not yaml_path.exists():
            raise FileNotFoundError(f"Could not find YAML: {root}.yaml or {root}.yml")

    meta = load_yaml(yaml_path)
    # Extract fields with defaults/sanity
    resolution = float(meta.get("resolution", 0.05))
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    x0, y0, yaw = float(origin[0]), float(origin[1]), float(origin[2]) if len(origin) > 2 else 0.0
    negate = int(meta.get("negate", 0))

    # Determine image path: prefer YAML's 'image' (may include relative dirs); fall back to <root>.pgm
    img_field = meta.get("image", f"{root.name}.pgm")
    # Resolve path relative to YAML file’s directory
    img_path = (yaml_path.parent / img_field).resolve()
    if not img_path.exists():
        img_path = root.with_suffix(".pgm")
        if not img_path.exists():
            raise FileNotFoundError(f"Could not find PGM image at '{(yaml_path.parent / img_field)}' or '{root}.pgm'")

    # Load image
    arr = load_pgm(img_path)
    H, W = arr.shape[:2]

    # Respect negate for visualization only
    if negate == 1:
        arr = 255 - arr

    # Flip vertically so +Y is up (imshow origin='lower')
    arr = np.flipud(arr)

    # Compute world extents in meters
    xmin = x0
    xmax = x0 + W * resolution
    ymin = y0
    ymax = y0 + H * resolution

    # Warn if yaw != 0 (unsupported rotation in this simple renderer)
    if abs(yaw) > 1e-6:
        print(f"[WARN] origin yaw={yaw:.6f} rad not applied; image rendered axis-aligned.")

    # Choose a figure size that honors aspect ratio and leaves room for axes
    # Base figure width on world width; clamp to a reasonable range
    world_w = xmax - xmin
    world_h = ymax - ymin
    aspect = world_w / world_h if world_h > 0 else 1.0
    fig_w = 8.0  # inches; you can tweak this
    fig_h = max(4.0, fig_w / max(aspect, 1e-6))

    fig, ax = plt.subplots(figsize=(fig_w, fig_h), constrained_layout=True)
    im = ax.imshow(arr, cmap="gray", vmin=0, vmax=255,
                   origin="lower", extent=[xmin, xmax, ymin, ymax], interpolation="nearest")

    # Axes/labels
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(root.name)

    if args.grid:
        ax.grid(True, which="both", color="#cccccc", linewidth=0.5, alpha=0.4)

    # Save
    out_path = root.with_suffix(".png")
    fig.savefig(out_path, dpi=args.dpi)
    plt.close(fig)
    print(f"Wrote {out_path}  [{W}×{H} px, res={resolution} m/px, origin=({x0:.3f},{y0:.3f},{yaw:.3f})]")

if __name__ == "__main__":
    main()
