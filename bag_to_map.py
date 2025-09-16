#!/usr/bin/env python3
import argparse
import math
import os
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

UNKNOWN_GRAY = 205  # standard trinary gray used by map_saver
MAX_PGM = 255

def yaw_from_quat(x, y, z, w):
    # Z-up yaw from geometry_msgs/Quaternion
    # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def read_first_msg_from_bag(bag_dir, topic_name, storage_id, serialization="cdr"):
    reader = SequentialReader()
    storage = StorageOptions(uri=bag_dir, storage_id=storage_id)
    converter = ConverterOptions(input_serialization_format=serialization,
                                 output_serialization_format=serialization)
    reader.open(storage, converter)

    # Find topic type
    target_type = None
    for t in reader.get_all_topics_and_types():
        if t.name == topic_name:
            target_type = t.type
            break
    if target_type is None:
        raise RuntimeError(f"Topic '{topic_name}' not found in bag: {bag_dir}")

    MsgType = get_message(target_type)

    # Iterate until we hit the first message for this topic
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, MsgType)
            return msg

    raise RuntimeError(f"No messages found on topic '{topic_name}' in bag: {bag_dir}")

def occ_to_pgm_bytes(grid, occ_thresh=0.65, free_thresh=0.25, negate=0, mode="trinary"):
    """
    Convert nav_msgs/OccupancyGrid to PGM bytes (P5).
    - mode='trinary' => occupied=0, free=254/255, unknown=205
    - We flip vertically so the saved image matches Nav2/map_saver convention.
    """
    w = grid.info.width
    h = grid.info.height
    data = grid.data  # sequence of int8 [-1..100], row-major, index = y*w + x

    # Precompute thresholds in 0..100
    occ_t = int(round(occ_thresh * 100.0))
    free_t = int(round(free_thresh * 100.0))

    pixels = bytearray(w * h)

    # Build pixel buffer with vertical flip (write top row first)
    for y_out, y in enumerate(reversed(range(h))):
        row_offset_out = y_out * w
        row_offset_in = y * w
        for x in range(w):
            v = data[row_offset_in + x]
            if mode == "trinary":
                if v < 0:               # unknown
                    p = UNKNOWN_GRAY
                elif v >= occ_t:        # occupied
                    p = 0
                elif v <= free_t:       # free
                    p = MAX_PGM
                else:                   # between thresholds -> unknown gray
                    p = UNKNOWN_GRAY
            else:  # 'raw' or 'scale' styles not needed here; keep trinary behavior
                p = UNKNOWN_GRAY

            # negate flips black/white (leave unknown as mid-gray)
            if negate:
                if p == 0:
                    p = MAX_PGM
                elif p == MAX_PGM:
                    p = 0

            pixels[row_offset_out + x] = p

    header = f"P5\n{w} {h}\n{MAX_PGM}\n".encode("ascii")
    return header + bytes(pixels)

def write_yaml(yaml_path, image_name, resolution, origin_xyz_yaw,
               negate=0, occ_thresh=0.65, free_thresh=0.25, mode="trinary"):
    ox, oy, yaw = origin_xyz_yaw
    # Minimal, Nav2-friendly YAML
    text = (
        f"image: {image_name}\n"
        f"mode: {mode}\n"
        f"resolution: {resolution:.9f}\n"
        f"origin: [{ox:.9f}, {oy:.9f}, {yaw:.9f}]\n"
        f"negate: {int(negate)}\n"
        f"occupied_thresh: {occ_thresh}\n"
        f"free_thresh: {free_thresh}\n"
    )
    with open(yaml_path, "w") as f:
        f.write(text)

def main():
    ap = argparse.ArgumentParser(description="Extract first OccupancyGrid from rosbag2 to Nav2 map (PGM+YAML).")
    ap.add_argument("--bag", required=True, help="Path to rosbag2 directory (the folder, not the .db3/.mcap file)")
    ap.add_argument("--topic", required=True, help="Topic name of the OccupancyGrid (e.g., /map)")
    ap.add_argument("--storage-id", default="sqlite3", help="rosbag2 storage plugin id (e.g., sqlite3, mcap)")
    ap.add_argument("--out", default="map", help="Output basename (e.g., 'map' -> map.pgm, map.yaml)")
    ap.add_argument("--mode", default="trinary", choices=["trinary"], help="Nav2 map mode (trinary)")
    ap.add_argument("--occupied-thresh", type=float, default=0.65)
    ap.add_argument("--free-thresh", type=float, default=0.25)
    ap.add_argument("--negate", type=int, default=0, choices=[0,1])
    args = ap.parse_args()

    grid = read_first_msg_from_bag(args.bag, args.topic, args.storage_id)
    # Pull metadata
    res = float(grid.info.resolution)
    w, h = grid.info.width, grid.info.height
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    q = grid.info.origin.orientation
    yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    # Write PGM
    pgm_bytes = occ_to_pgm_bytes(
        grid,
        occ_thresh=args.occupied_thresh,
        free_thresh=args.free_thresh,
        negate=args.negate,
        mode=args.mode,
    )

    pgm_path = f"{args.out}.pgm"
    with open(pgm_path, "wb") as f:
        f.write(pgm_bytes)

    # Write YAML (use relative image name)
    yaml_path = f"{args.out}.yaml"
    image_name = os.path.basename(pgm_path)
    write_yaml(yaml_path, image_name, res, (ox, oy, yaw),
               negate=args.negate,
               occ_thresh=args.occupied_thresh,
               free_thresh=args.free_thresh,
               mode=args.mode)

    print(f"Wrote:\n  {pgm_path}  ({w}x{h})\n  {yaml_path}")

if __name__ == "__main__":
    main()
