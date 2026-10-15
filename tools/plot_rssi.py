#!/usr/bin/env python3
"""Compare RSSI filters on receiver CSV logs, one log file per true distance.

Usage:
    python tools/plot_rssi.py logs/1m.csv logs/1.5m.csv logs/2m.csv

The true distance is read from the file name (1m.csv, 1.5m.csv, rssi_2_5m.csv).
Needs: pip install numpy pandas matplotlib
"""
import argparse
import math
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

FILTER_NAMES = ["Raw Rssi", "Median", "Moving Average", "Kalman",
                "Hybrid Gaussian Kalman", "SNR Adaptive Kalman"]
COLORS = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE"]

PROCESS_NOISE = 4.0      # same Q as the firmware Kalman filter (dB^2)
MEASUREMENT_NOISE = 9.0  # same R as the firmware Kalman filter (dB^2)
WINDOW = 3               # firmware median / moving-average window


def median(rssi):
    out = []
    for i in range(len(rssi)):
        window = np.sort(rssi[max(0, i - WINDOW + 1): i + 1])
        out.append(window[len(window) // 2])
    return np.array(out)


def moving_average(rssi):
    return np.array([rssi[max(0, i - WINDOW + 1): i + 1].mean() for i in range(len(rssi))])


def snr_adaptive_variance(snr, lqi):
    if np.isnan(snr) or snr < 3 or lqi < 64:
        return 100.0
    if snr < 8 or lqi < 128:
        return 49.0
    if snr < 15 or lqi < 192:
        return 25.0
    return 9.0


def kalman(rssi, variances=None, hybrid=False):
    if variances is None:
        variances = np.full(len(rssi), MEASUREMENT_NOISE)
    estimate, covariance = rssi[0], variances[0]
    out = [estimate]
    for z, noise in zip(rssi[1:], variances[1:]):
        if hybrid:
            # Same bell curve as the firmware's gaussian_likelihood table (sigma = 4 dB).
            likelihood = max(math.exp(-((z - estimate) ** 2) / 32.0), 1.0 / 255.0)
            noise = min(noise / likelihood, 1000.0)
        prior = covariance + PROCESS_NOISE
        gain = prior / (prior + noise)
        estimate += gain * (z - estimate)
        covariance = (1.0 - gain) * prior
        out.append(estimate)
    return np.array(out)


def apply_filters(group):
    rssi = group["raw_dbm"].to_numpy(dtype=float)
    snr = group["snr_est_db"].to_numpy(dtype=float)
    lqi = group["lqi"].fillna(0).to_numpy(dtype=float)
    adaptive = np.array([snr_adaptive_variance(s, q) for s, q in zip(snr, lqi)])
    return [
        rssi,
        median(rssi),
        moving_average(rssi),
        kalman(rssi),
        kalman(rssi, hybrid=True),
        kalman(rssi, variances=adaptive),
    ]


def distance_from_filename(path):
    match = re.search(r"(\d+(?:[._]\d+)?)\s*m", path.stem)
    if not match:
        raise ValueError(f"cannot find the distance in '{path.name}' (use names like 1.5m.csv)")
    return float(match.group(1).replace("_", "."))


def load_log(path):
    frame = pd.read_csv(path, on_bad_lines="skip")
    missing = {"status", "raw_dbm"} - set(frame.columns)
    if missing:
        raise ValueError(f"missing columns {sorted(missing)}; expected the receiver CSV format")
    frame = frame[frame["status"] == "beacon"].copy()
    for column in ("raw_dbm", "lqi", "snr_est_db"):
        frame[column] = pd.to_numeric(frame[column], errors="coerce") if column in frame else np.nan
    return frame.dropna(subset=["raw_dbm"])


def filtered_rssi_per_filter(frame):
    groups = [g for _, g in frame.groupby("vehicle_id", sort=False)] if "vehicle_id" in frame else [frame]
    per_filter = [[] for _ in FILTER_NAMES]
    for group in groups:
        raw = group["raw_dbm"].to_numpy(dtype=float)
        if len(raw) == 0:
            continue
        for bucket, values in zip(per_filter, apply_filters(group)):
            bucket.append(values)
    return [np.concatenate(b) if b else np.array([]) for b in per_filter]


def calibrate(distances, raw_by_distance, rssi_at_1m, path_loss_exp):
    if rssi_at_1m is not None and path_loss_exp is not None:
        return rssi_at_1m, path_loss_exp
    xs, ys = [], []
    for distance in distances:
        values = raw_by_distance[distance]
        xs.extend([math.log10(distance)] * len(values))
        ys.extend(values)
    if len(set(distances)) >= 2:
        slope, intercept = np.polyfit(xs, ys, 1)
        if slope < 0:
            return (rssi_at_1m if rssi_at_1m is not None else intercept,
                    path_loss_exp if path_loss_exp is not None else -slope / 10.0)
    return (rssi_at_1m if rssi_at_1m is not None else -50.0,
            path_loss_exp if path_loss_exp is not None else 2.5)


def to_distance(rssi, rssi_at_1m, path_loss_exp):
    return 10.0 ** ((rssi_at_1m - rssi) / (10.0 * path_loss_exp))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+", type=Path, help="receiver CSV logs named like 1.5m.csv")
    parser.add_argument("--rssi-at-1m", type=float, help="RSSI at 1 m in dBm (default: fitted from the logs)")
    parser.add_argument("--path-loss-exp", type=float, help="path-loss exponent (default: fitted from the logs)")
    parser.add_argument("--output", type=Path, default=Path("filter_comparison.png"), help="image to save")
    parser.add_argument("--no-show", action="store_true", help="save the image without opening a window")
    args = parser.parse_args()

    filtered = {}
    for path in args.files:
        try:
            distance = distance_from_filename(path)
            filtered.setdefault(distance, [np.array([]) for _ in FILTER_NAMES])
            per_filter = filtered_rssi_per_filter(load_log(path))
        except (ValueError, OSError) as error:
            sys.exit(f"{path}: {error}")
        if len(per_filter[0]) == 0:
            sys.exit(f"{path}: no beacon rows found")
        filtered[distance] = [np.concatenate([a, b]) for a, b in zip(filtered[distance], per_filter)]

    distances = sorted(filtered)
    rssi_at_1m, path_loss_exp = calibrate(distances, {d: filtered[d][0] for d in distances},
                                          args.rssi_at_1m, args.path_loss_exp)
    print(f"RSSI at 1 m = {rssi_at_1m:.1f} dBm, path-loss exponent = {path_loss_exp:.2f}")

    means = np.zeros((len(FILTER_NAMES), len(distances)))
    stds = np.zeros_like(means)
    for j, distance in enumerate(distances):
        for i, values in enumerate(filtered[distance]):
            estimates = to_distance(values, rssi_at_1m, path_loss_exp)
            means[i, j], stds[i, j] = estimates.mean(), estimates.std()
        print(f"{distance:g} m: " + ", ".join(
            f"{name} {means[i, j]:.2f}+-{stds[i, j]:.2f}" for i, name in enumerate(FILTER_NAMES)))

    fig, ax = plt.subplots(figsize=(8, 6), dpi=100)
    positions = np.arange(len(distances))
    width = 0.13
    for i, name in enumerate(FILTER_NAMES):
        ax.bar(positions + (i - (len(FILTER_NAMES) - 1) / 2) * width, means[i], width,
               yerr=stds[i], capsize=3, label=name, color=COLORS[i], edgecolor="black", linewidth=0.5,
               error_kw={"ecolor": "black", "elinewidth": 1.2})
    ax.set_xticks(positions)
    ax.set_xticklabels([f"{d:g}" for d in distances])
    ax.set_xlabel("Distance(meters)", fontsize=20)
    ax.set_ylabel("Results(meters)", fontsize=20)
    ax.set_ylim(0, max(9.0, math.ceil((means + stds).max() * 1.45)))
    ax.legend(loc="upper left", fontsize=14)
    fig.tight_layout()
    fig.savefig(args.output)
    print(f"saved {args.output}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
