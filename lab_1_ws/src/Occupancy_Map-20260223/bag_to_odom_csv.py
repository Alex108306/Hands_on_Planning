#!/usr/bin/env python3
"""
Extract nav_msgs/msg/Odometry from a ROS 2 bag into a CSV file
matching the format of ground_truth_02.csv.

Usage:
  source /opt/ros/<distro>/setup.bash
  python3 bag_to_odom_csv.py --bag /path/to/bag --topic /odom --out ground_truth.csv

If your bag uses MCAP storage, add:  --storage mcap
"""
import csv
import argparse

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser(description="ROS 2 bag Odometry → CSV")
    parser.add_argument("--bag", required=True, help="Path to the ros2 bag directory or file")
    parser.add_argument("--topic", default="/odom", help="Odometry topic name (default: /odom)")
    parser.add_argument("--out", default="ground_truth.csv", help="Output CSV path")
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

    # Build header
    cols = [
        "%time",
        "field.header.seq",
        "field.header.stamp",
        "field.header.frame_id",
        "field.child_frame_id",
        "field.pose.pose.position.x",
        "field.pose.pose.position.y",
        "field.pose.pose.position.z",
        "field.pose.pose.orientation.x",
        "field.pose.pose.orientation.y",
        "field.pose.pose.orientation.z",
        "field.pose.pose.orientation.w",
    ]
    cols += [f"field.pose.covariance{i}" for i in range(36)]
    cols += [
        "field.twist.twist.linear.x",
        "field.twist.twist.linear.y",
        "field.twist.twist.linear.z",
        "field.twist.twist.angular.x",
        "field.twist.twist.angular.y",
        "field.twist.twist.angular.z",
    ]
    cols += [f"field.twist.covariance{i}" for i in range(36)]

    row_count = 0

    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(cols)

        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic != args.topic:
                continue

            msg = deserialize_message(data, msg_type)
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

            row = [
                t,
                getattr(msg.header, "seq", 0),
                stamp_ns,
                msg.header.frame_id,
                msg.child_frame_id,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
            row += list(msg.pose.covariance)
            row += [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ]
            row += list(msg.twist.covariance)

            writer.writerow(row)
            row_count += 1

    print(f"Done – wrote {row_count} messages to {args.out}")


if __name__ == "__main__":
    main()
