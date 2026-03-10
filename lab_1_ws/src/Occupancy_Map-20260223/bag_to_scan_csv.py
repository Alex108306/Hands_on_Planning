#!/usr/bin/env python3
"""
Extract sensor_msgs/msg/LaserScan from a ROS 2 bag into a CSV file
matching the format of laser_scan_02.csv.

Usage:
  source /opt/ros/<distro>/setup.bash
  python3 bag_to_scan_csv.py --bag /path/to/your_bag --topic /scan --out laser_scan.csv

If your bag uses MCAP storage, add:  --storage mcap
"""
import csv
import math
import argparse

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def float_str(v):
    if isinstance(v, float):
        if math.isinf(v):
            return "inf" if v > 0 else "-inf"
        if math.isnan(v):
            return "nan"
    return str(v)


def main():
    parser = argparse.ArgumentParser(description="ROS 2 bag LaserScan → CSV")
    parser.add_argument("--bag", required=True, help="Path to the ros2 bag directory")
    parser.add_argument("--topic", default="/scan", help="LaserScan topic name (default: /scan)")
    parser.add_argument("--out", default="laser_scan.csv", help="Output CSV path")
    parser.add_argument("--storage", default="sqlite3", help="Storage plugin: sqlite3 or mcap")
    args = parser.parse_args()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id=args.storage),
        ConverterOptions("", ""),
    )

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in type_map:
        available = ", ".join(type_map.keys())
        raise SystemExit(f"Topic '{args.topic}' not in bag. Available: {available}")

    msg_type = get_message(type_map[args.topic])
    header_written = False
    row_count = 0

    with open(args.out, "w", newline="") as f:
        writer = None
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic != args.topic:
                continue

            msg = deserialize_message(data, msg_type)

            if not header_written:
                cols = [
                    "%time",
                    "field.header.seq",
                    "field.header.stamp",
                    "field.header.frame_id",
                    "field.angle_min",
                    "field.angle_max",
                    "field.angle_increment",
                    "field.time_increment",
                    "field.scan_time",
                    "field.range_min",
                    "field.range_max",
                ]
                cols += [f"field.ranges{i}" for i in range(len(msg.ranges))]
                cols += [f"field.intensities{i}" for i in range(len(msg.intensities))]
                writer = csv.writer(f)
                writer.writerow(cols)
                header_written = True

            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

            row = [
                t,                                       # %time  (bag timestamp, ns)
                getattr(msg.header, "seq", 0),           # seq (always 0 in ROS 2)
                stamp_ns,                                # header.stamp as ns
                msg.header.frame_id,
                float_str(msg.angle_min),
                float_str(msg.angle_max),
                float_str(msg.angle_increment),
                float_str(msg.time_increment),
                float_str(msg.scan_time),
                float_str(msg.range_min),
                float_str(msg.range_max),
            ]
            row += [float_str(v) for v in msg.ranges]
            row += [float_str(v) for v in msg.intensities]

            writer.writerow(row)
            row_count += 1

    print(f"Done – wrote {row_count} messages to {args.out}")


if __name__ == "__main__":
    main()
