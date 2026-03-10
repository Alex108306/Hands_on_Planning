#!/usr/bin/env python3
"""
Extract & filter sensor_msgs/msg/LaserScan from a ROS 2 bag into CSV.

Filtering applied:
  1. Clamp ranges to [range_min, range_max]; out-of-bound → inf
  2. Median filter (sliding window) across neighbouring beams to remove spikes
  3. Small inf-gaps (≤ gap_max consecutive infs) are linearly interpolated

Usage:
  python3 bag_to_scan_csv_filtered.py \
      --bag /path/to/bag --topic /scan --storage mcap \
      --median-window 5 --gap-max 3 \
      --out laser_scan_filtered.csv
"""
import csv
import math
import argparse
import statistics

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


def median_filter(ranges, window):
    """Apply a sliding-window median filter to the range array."""
    n = len(ranges)
    if window < 3 or n == 0:
        return list(ranges)
    half = window // 2
    out = list(ranges)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        # Only use finite values in the median computation
        vals = [ranges[j] for j in range(lo, hi) if math.isfinite(ranges[j])]
        if vals:
            out[i] = statistics.median(vals)
        # else leave as-is (inf)
    return out


def interpolate_small_gaps(ranges, gap_max):
    """Linearly interpolate short runs of inf (≤ gap_max consecutive)."""
    n = len(ranges)
    out = list(ranges)
    i = 0
    while i < n:
        if not math.isfinite(out[i]):
            # find run of non-finite values
            j = i
            while j < n and not math.isfinite(out[j]):
                j += 1
            gap_len = j - i
            # interpolate only if gap is small and bounded by finite values
            if gap_len <= gap_max and i > 0 and j < n:
                left = out[i - 1]
                right = out[j]
                if math.isfinite(left) and math.isfinite(right):
                    for k in range(i, j):
                        t = (k - i + 1) / (gap_len + 1)
                        out[k] = left + t * (right - left)
            i = j
        else:
            i += 1
    return out


def filter_scan(ranges, range_min, range_max, median_win, gap_max):
    """Full filtering pipeline for one scan."""
    # 1. Clamp out-of-bound readings to inf
    clamped = []
    for v in ranges:
        if math.isfinite(v) and range_min <= v <= range_max:
            clamped.append(v)
        else:
            clamped.append(float("inf"))
    # 2. Median filter
    filtered = median_filter(clamped, median_win)
    # 3. Interpolate small gaps
    filtered = interpolate_small_gaps(filtered, gap_max)
    return filtered


def main():
    parser = argparse.ArgumentParser(description="ROS 2 bag LaserScan → filtered CSV")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--out", default="laser_scan_filtered.csv")
    parser.add_argument("--storage", default="sqlite3")
    parser.add_argument("--median-window", type=int, default=5,
                        help="Median filter window size (odd, ≥3). 0 = disabled.")
    parser.add_argument("--gap-max", type=int, default=3,
                        help="Max consecutive inf gap to interpolate. 0 = disabled.")
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

            # --- apply filtering ---
            filtered_ranges = filter_scan(
                list(msg.ranges),
                msg.range_min,
                msg.range_max,
                args.median_window,
                args.gap_max,
            )

            row = [
                t,
                getattr(msg.header, "seq", 0),
                stamp_ns,
                msg.header.frame_id,
                float_str(msg.angle_min),
                float_str(msg.angle_max),
                float_str(msg.angle_increment),
                float_str(msg.time_increment),
                float_str(msg.scan_time),
                float_str(msg.range_min),
                float_str(msg.range_max),
            ]
            row += [float_str(v) for v in filtered_ranges]
            row += [float_str(v) for v in msg.intensities]

            writer.writerow(row)
            row_count += 1

    print(f"Done – wrote {row_count} filtered scan messages to {args.out}")


if __name__ == "__main__":
    main()
