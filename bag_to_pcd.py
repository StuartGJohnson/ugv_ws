#!/usr/bin/env python3
import os
import sys
import argparse
import struct
import tempfile
from typing import Optional, Tuple, List
import numpy as np

# ROS 2 bag + serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# PointCloud2 helpers
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2

# Optional: octomap Python bindings for ColorOcTree parsing
import octomap
# try:
#     import octomap  # sudo apt install ros-humble-octomap? (bindings may be packaged as python3-octomap)
#     HAVE_OCTOMAP = True
# except Exception:
#     HAVE_OCTOMAP = False


def open_reader(uri: str) -> rosbag2_py.SequentialReader:
    """
    Try to open the bag with common storages (sqlite3, mcap).
    """
    storage_ids = ['mcap']
    converter = rosbag2_py.ConverterOptions('', '')  # no conversion
    for sid in storage_ids:
        try:
            reader = rosbag2_py.SequentialReader()
            storage = rosbag2_py.StorageOptions(uri=uri, storage_id=sid)
            reader.open(storage, converter)
            return reader
        except Exception:
            pass
    raise RuntimeError(f"Failed to open bag at '{uri}' with sqlite3 or mcap.")


def find_message(reader: rosbag2_py.SequentialReader,
                 topic_name: str) -> Tuple[bytes, str]:
    """
    Scan the bag and return (serialized, type_str) for the first occurrence of `topic_name`.
    Resets the reader by reopening (rosbag2_py has forward-only iteration).
    """
    # We must reopen to scan from the start; so capture options then re-open.
    # Unfortunately SequentialReader doesn't expose options after open.
    # Workaround: we re-open a new reader with the same URI by reading metadata.yaml.
    # Simpler: open a fresh reader each time.
    uri = reader._reader.impl.get_current_file()  # internal, may not exist across all distros
    # Fallback: just try opening a new reader with stored args
    # Because the impl accessor is brittle, we’ll instead accept scanning once across both topics
    # -> New approach: scan once for both topics.
    raise NotImplementedError


def read_first_messages(uri: str,
                        want_topics: List[str]) -> Tuple[dict, dict]:
    """
    Read through the bag ONCE and collect the first serialized samples for all `want_topics`.
    Returns (topic->(serialized, type_str), types_map topic->typename).
    """
    reader = open_reader(uri)
    all_meta = {m.name: m.type for m in reader.get_all_topics_and_types()}
    want_set = set(want_topics)
    found = {}
    types = {}
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic in want_set and topic not in found:
            types[topic] = all_meta.get(topic, '')
            found[topic] = (data, types[topic])
        if len(found) == len(want_set):
            break
    return found, types


def pack_rgb_float(r: int, g: int, b: int) -> float:
    """Pack 3x uint8 into PCL-style float32 RGB field."""
    i = ((r & 0xff) << 16) | ((g & 0xff) << 8) | (b & 0xff)
    return struct.unpack('f', struct.pack('I', i))[0]


def write_pcd_xyz(path: str, pts) -> None:
    """Write XYZ ASCII PCD."""
    n = len(pts)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )
    with open(path, 'w') as f:
        f.write(header)
        for x, y, z, *_ in pts:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def write_pcd_xyzrgb(path: str, pts_rgb) -> None:
    """Write XYZ+RGB (float packed) ASCII PCD. pts_rgb: iterable of (x,y,z,r,g,b)."""
    n = len(pts_rgb)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA ascii\n"
    )
    with open(path, 'w') as f:
        f.write(header)
        for x, y, z, r, g, b in pts_rgb:
            rgbf = pack_rgb_float(int(r), int(g), int(b))
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {rgbf:.8f}\n")


def cloud_to_pcd(serialized: bytes, type_str: str, out_path: str) -> None:
    """Deserialize a sensor_msgs/PointCloud2 and dump to PCD (XYZ or XYZRGB)."""
    msg_cls = get_message(type_str)
    msg: PointCloud2 = deserialize_message(serialized, msg_cls)

    has_rgb = any(f.name in ('rgb', 'rgba') for f in msg.fields)
    has_rgb = False
    if has_rgb:
        print('rgb detect')
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
        xyzrgb = []
        for x, y, z, rgb in pts:
            # rgb may arrive as float32 (packed)
            if isinstance(rgb, float):
                i = struct.unpack('I', struct.pack('f', rgb))[0]
                r = (i >> 16) & 0xff
                g = (i >> 8) & 0xff
                b = i & 0xff
            else:
                # If it's already int or tuple (rare), try best-effort
                r = g = b = int(rgb) & 0xff
            xyzrgb.append((x, y, z, r, g, b))
        # ros2 PointCloud2 messages seem to have bogus color encodings
        # write_pcd_xyzrgb(out_path, xyzrgb)
        write_pcd_xyz(out_path, xyzrgb)
    else:
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = [(x, y, z) for (x, y, z) in pts]
        write_pcd_xyz(out_path, xyz)


def octomap_color_to_points(octomap_msg, max_depth_only=True) -> List[Tuple[float, float, float, int, int, int]]:
    """
    Convert a ColorOcTree (octomap_msgs/Octomap, id='ColorOcTree') to colored points.
    Requires octomap Python bindings.
    Strategy:
      - Write msg.data to a temp .bt file, read it with octomap.readBinary()
      - Iterate occupied leaf nodes and extract (x,y,z) + color
    """

    if getattr(octomap_msg, 'id', '') != 'ColorOcTree':
        raise RuntimeError(f"OctoMap id is '{octomap_msg.id}', expected 'ColorOcTree'.")

    # Write raw binary stream to a temp .bt file
    with tempfile.NamedTemporaryFile(prefix='octomap_', suffix='.bt', delete=False) as tf:
        tf.write(bytes(octomap_msg.data))
        bt_path = tf.name

    tree = octomap.readBinary(bt_path)  # returns AbstractOcTree
    os.unlink(bt_path)

    # Attempt to cast to ColorOcTree
    if not isinstance(tree, octomap.ColorOcTree):
        # Some bindings return a generic but behave like ColorOcTree; try attribute presence
        if not hasattr(tree, 'isNodeOccupied') and not hasattr(tree, 'getTreeType'):
            raise RuntimeError("Loaded octomap is not a ColorOcTree (cannot find color API).")

    pts = []
    # Iterate over leafs; Python API differs by version. Try common patterns.
    # Fallback: use bounding box iteration over entire space if available.
    try:
        it = tree.begin_leafs()
        end = tree.end_leafs()
        while it != end:
            node = it.getNode() if hasattr(it, 'getNode') else it  # some APIs yield the node
            if tree.isNodeOccupied(node):
                x = it.getX(); y = it.getY(); z = it.getZ()
                if hasattr(tree, 'isNodeAtThreshold') and not tree.isNodeAtThreshold(node):
                    pass  # optional filtering
                # Color
                if hasattr(node, 'getColor'):
                    c = node.getColor()
                    r, g, b = int(c.r), int(c.g), int(c.b)
                elif hasattr(it, 'getColor'):
                    c = it.getColor()
                    r, g, b = int(c.r), int(c.g), int(c.b)
                else:
                    r = g = b = 200
                pts.append((x, y, z, r, g, b))
            it.__iadd__(1)  # ++it
    except Exception:
        # Very binding-dependent; try scanning a big BBX at max depth
        try:
            # Derive a loose bbox from tree bounding box (if exposed)
            # Some bindings: tree.getMetricMin(x,y,z) / getMetricMax(...)
            import math
            minx=miny=minz= -1000.0
            maxx=maxy=maxz= 1000.0
            if hasattr(tree, 'getMetricMin'):
                x=y=z=0.0
                minx, miny, minz = tree.getMetricMin()
                maxx, maxy, maxz = tree.getMetricMax()
            it = tree.begin_leafs_bbx((minx, miny, minz), (maxx, maxy, maxz))
            end = tree.end_leafs_bbx()
            while it != end:
                node = it.getNode() if hasattr(it, 'getNode') else it
                if tree.isNodeOccupied(node):
                    x = it.getX(); y = it.getY(); z = it.getZ()
                    if hasattr(node, 'getColor'):
                        c = node.getColor()
                        r, g, b = int(c.r), int(c.g), int(c.b)
                    else:
                        r = g = b = 200
                    pts.append((x, y, z, r, g, b))
                it.__iadd__(1)
        except Exception as e2:
            raise RuntimeError(f"OctoMap traversal failed; Python bindings vary. {e2}")

    return pts


def octomap_to_pcd(serialized: bytes, type_str: str, out_path: str, also_write_bt: Optional[str] = None) -> None:
    """
    Deserialize octomap_msgs/Octomap (ColorOcTree) and dump colored points to PCD.
    If Python octomap bindings are missing, write raw .bt file so it can be post-processed.
    """
    msg_cls = get_message(type_str)
    msg = deserialize_message(serialized, msg_cls)

    if getattr(msg, 'id', '') != 'ColorOcTree':
        raise RuntimeError(f"OctoMap message id is '{msg.id}', expected 'ColorOcTree'.")

    # if not HAVE_OCTOMAP:
    #     # Fallback: dump raw binary .bt for later handling
    #     bt_path = also_write_bt or (os.path.splitext(out_path)[0] + '.bt')
    #     with open(bt_path, 'wb') as f:
    #         f.write(bytes(msg.data))
    #     print(f"[octomap] Python bindings unavailable. Wrote raw binary to: {bt_path}")
    #     print("          Use octomap tools to convert to point cloud later.")
    #     return

    pts_rgb = octomap_color_to_points(msg)
    write_pcd_xyzrgb(out_path, pts_rgb)


def main():
    ap = argparse.ArgumentParser(description="Extract PointCloud2 and ColorOcTree from a rosbag2 snapshot and write PCD.")
    ap.add_argument('bag', help='Path to rosbag2 directory (containing metadata.yaml)')
    ap.add_argument('--cloud-topic', default=None, help='PointCloud2 topic name')
    ap.add_argument('--cloud-out', default='/tmp/cloud.pcd', help='Output PCD path for the cloud')
    ap.add_argument('--octomap-topic', default=None, help='OctoMap topic name (ColorOcTree)')
    ap.add_argument('--octomap-out', default='/tmp/octomap_points.pcd', help='Output PCD path for octomap colored points')
    args = ap.parse_args()

    want = []
    if args.cloud_topic:
        want.append(args.cloud_topic)
    if args.octomap_topic:
        want.append(args.octomap_topic)
    if not want:
        print("Nothing to do: specify --cloud-topic and/or --octomap-topic")
        sys.exit(1)

    found, types = read_first_messages(args.bag, want)

    if args.cloud_topic:
        if args.cloud_topic not in found:
            print(f"[cloud] Topic '{args.cloud_topic}' not found in bag.")
        else:
            ser, tstr = found[args.cloud_topic]
            if tstr not in ('sensor_msgs/msg/PointCloud2', 'sensor_msgs/PointCloud2'):
                print(f"[cloud] Topic '{args.cloud_topic}' is type '{tstr}', expected PointCloud2.")
            else:
                print(f"[cloud] Writing PCD -> {args.cloud_out}")
                cloud_to_pcd(ser, tstr, args.cloud_out)

    if args.octomap_topic:
        if args.octomap_topic not in found:
            print(f"[octomap] Topic '{args.octomap_topic}' not found in bag.")
        else:
            ser, tstr = found[args.octomap_topic]
            if tstr not in ('octomap_msgs/msg/Octomap', 'octomap_msgs/Octomap'):
                print(f"[octomap] Topic '{args.octomap_topic}' is type '{tstr}', expected octomap_msgs/Octomap.")
            else:
                print(f"[octomap] Writing colored PCD -> {args.octomap_out}")
                try:
                    octomap_to_pcd(ser, tstr, args.octomap_out)
                except Exception as e:
                    # Still write raw .bt to help the user
                    bt_fallback = os.path.splitext(args.octomap_out)[0] + '.bt'
                    print(f"[octomap] Error decoding ColorOcTree: {e}")
                    try:
                        msg_cls = get_message(tstr)
                        msg = deserialize_message(ser, msg_cls)
                        with open(bt_fallback, 'wb') as f:
                            f.write(bytes(msg.data))
                        print(f"[octomap] Wrote raw binary to: {bt_fallback}")
                    except Exception:
                        pass

    print("Done.")


if __name__ == '__main__':
    main()
