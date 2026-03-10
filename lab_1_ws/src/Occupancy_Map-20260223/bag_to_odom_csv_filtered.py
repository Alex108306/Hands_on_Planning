#!/usr/bin/env python3
"""
Extract & filter nav_msgs/msg/Odometry from a ROS 2 bag into CSV.

Filtering applied:
  - Exponential Moving Average (EMA) on position (x, y, z)
  - SLERP-based EMA on orientation quaternion (keeps it normalised)
  - EMA on twist (linear & angular velocities)

Alpha controls smoothness: 0 < alpha ≤ 1.
  alpha = 1.0  → no filtering (raw data)
  alpha = 0.1  → very smooth / heavy filtering

Usage:
  python3 bag_to_odom_csv_filtered.py \
      --bag /path/to/bag --topic /odom --storage mcap \
      --alpha 0.3 \
      --out ground_truth_filtered.csv
"""
import csv
import math
import argparse

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ── quaternion helpers ──────────────────────────────────────────────
def quat_dot(a, b):
    return sum(ai * bi for ai, bi in zip(a, b))


def quat_normalize(q):
    n = math.sqrt(quat_dot(q, q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(c / n for c in q)


def quat_slerp(q0, q1, t):
    """Spherical linear interpolation between two unit quaternions."""
    dot = quat_dot(q0, q1)
    # Ensure shortest path
    if dot < 0.0:
        q1 = tuple(-c for c in q1)
        dot = -dot
    dot = min(dot, 1.0)

    if dot > 0.9995:
        # Very close – use linear interpolation and normalise
        result = tuple(a + t * (b - a) for a, b in zip(q0, q1))
        return quat_normalize(result)

    theta_0 = math.acos(dot)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return quat_normalize(tuple(s0 * a + s1 * b for a, b in zip(q0, q1)))


# ── EMA state ──────────────────────────────────────────────────────
class OdomFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.pos = None   # (x, y, z)
        self.quat = None  # (x, y, z, w)
        self.lin = None   # (vx, vy, vz)
        self.ang = None   # (wx, wy, wz)

    def update(self, pos, quat, lin, ang):
        a = self.alpha
        if self.pos is None:
            self.pos = pos
            self.quat = quat
            self.lin = lin
            self.ang = ang
        else:
            self.pos = tuple(a * n + (1 - a) * o for n, o in zip(pos, self.pos))
            self.quat = quat_slerp(self.quat, quat, a)
            self.lin = tuple(a * n + (1 - a) * o for n, o in zip(lin, self.lin))
            self.ang = tuple(a * n + (1 - a) * o for n, o in zip(ang, self.ang))
        return self.pos, self.quat, self.lin, self.ang


def main():
    parser = argparse.ArgumentParser(description="ROS 2 bag Odometry → filtered CSV")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topic", default="/odom")
    parser.add_argument("--out", default="ground_truth_filtered.csv")
    parser.add_argument("--storage", default="sqlite3")
    parser.add_argument("--alpha", type=float, default=0.3,
                        help="EMA smoothing factor (0<α≤1). Lower = smoother. Default 0.3")
    args = parser.parse_args()

    if not 0 < args.alpha <= 1:
        raise SystemExit("--alpha must be in (0, 1]")

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
    filt = OdomFilter(args.alpha)

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

            p = msg.pose.pose
            tw = msg.twist.twist

            pos = (p.position.x, p.position.y, p.position.z)
            quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            lin = (tw.linear.x, tw.linear.y, tw.linear.z)
            ang = (tw.angular.x, tw.angular.y, tw.angular.z)

            f_pos, f_quat, f_lin, f_ang = filt.update(pos, quat, lin, ang)

            row = [
                t,
                getattr(msg.header, "seq", 0),
                stamp_ns,
                msg.header.frame_id,
                msg.child_frame_id,
                *f_pos,
                *f_quat,
            ]
            row += list(msg.pose.covariance)
            row += list(f_lin)
            row += list(f_ang)
            row += list(msg.twist.covariance)

            writer.writerow(row)
            row_count += 1

    print(f"Done – wrote {row_count} filtered odom messages to {args.out}")


if __name__ == "__main__":
    main()
