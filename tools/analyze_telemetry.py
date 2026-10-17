#!/usr/bin/env python3
"""Summarize and optionally flatten navigation JSONL telemetry."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

SENSOR_NAMES = ("ultrasonic_left", "ultrasonic_right", "infrared_left", "infrared_right")


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    index = fraction * (len(ordered) - 1)
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def load_records(path: Path) -> tuple[list[dict[str, Any]], int]:
    records: list[dict[str, Any]] = []
    ignored = 0
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                ignored += 1
                continue
            if "t_ms" in record and "state" in record:
                records.append(record)
            else:
                ignored += 1
    return records, ignored


def numeric(records: Iterable[dict[str, Any]], key: str) -> list[float]:
    return [float(record[key]) for record in records if isinstance(record.get(key), (int, float))]


def write_csv(records: list[dict[str, Any]], output: Path) -> None:
    fields = [
        "t_ms", *SENSOR_NAMES, *(f"{name}_health" for name in SENSOR_NAMES),
        "distance_cm", "confidence", "state", "direction", "ttc_s", "speed_limit_pct",
    ]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            row = {key: record.get(key) for key in fields}
            for name, value in zip(SENSOR_NAMES, record.get("sensors_cm", [])):
                row[name] = value
            for name, value in zip(SENSOR_NAMES, record.get("health", [])):
                row[f"{name}_health"] = value
            writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="JSONL file captured from serial output")
    parser.add_argument("--csv", type=Path, help="optional flattened CSV output path")
    args = parser.parse_args()

    records, ignored = load_records(args.input)
    if not records:
        parser.error("no navigation telemetry records found")

    timestamps = numeric(records, "t_ms")
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:]) if b >= a]
    distances = numeric(records, "distance_cm")
    confidence = numeric(records, "confidence")
    ttc = numeric(records, "ttc_s")
    states = Counter(str(record["state"]) for record in records)
    available = Counter()
    for record in records:
        for name, value in zip(SENSOR_NAMES, record.get("sensors_cm", [])):
            if value is not None:
                available[name] += 1

    print(f"records: {len(records)} ({ignored} ignored lines)")
    if intervals:
        print(f"cycle interval: median={statistics.median(intervals):.1f} ms, p95={percentile(intervals, 0.95):.1f} ms")
    print(f"range: min={min(distances):.1f} cm, median={statistics.median(distances):.1f} cm")
    print(f"confidence: mean={statistics.fmean(confidence):.2f}, p05={percentile(confidence, 0.05):.2f}")
    print("state occupancy: " + ", ".join(f"{state}={count}" for state, count in sorted(states.items())))
    if ttc:
        print(f"TTC events: {len(ttc)}, minimum={min(ttc):.2f} s")
    print("sensor availability: " + ", ".join(
        f"{name}={100.0 * available[name] / len(records):.1f}%" for name in SENSOR_NAMES
    ))

    if args.csv:
        write_csv(records, args.csv)
        print(f"wrote {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
