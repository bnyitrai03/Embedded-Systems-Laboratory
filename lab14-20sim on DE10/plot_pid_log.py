#!/usr/bin/env python3
import csv
import sys

import matplotlib.pyplot as plt


def read_log(path):
    with open(path, newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))

    if not rows:
        raise ValueError(f"{path} has no data rows")

    def column(name):
        return [float(row[name]) for row in rows]

    return {
        "time_s": column("time_s"),
        "yaw_target_rad": column("yaw_target_rad"),
        "pitch_target_rad": column("pitch_target_rad"),
        "yaw_actual_rad": column("yaw_actual_rad"),
        "pitch_actual_rad": column("pitch_actual_rad"),
        "yaw_error_rad": column("yaw_error_rad"),
        "pitch_error_rad": column("pitch_error_rad"),
        "yaw_output": column("yaw_output"),
        "pitch_output": column("pitch_output"),
        "yaw_pwm": column("yaw_pwm"),
        "pitch_pwm": column("pitch_pwm"),
        "lateness_us": column("lateness_us"),
    }


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "pid_log.csv"
    data = read_log(path)
    time_s = data["time_s"]

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 8))

    axes[0].plot(time_s, data["yaw_target_rad"], label="yaw target")
    axes[0].plot(time_s, data["yaw_actual_rad"], label="yaw actual")
    axes[0].plot(time_s, data["pitch_target_rad"], label="pitch target")
    axes[0].plot(time_s, data["pitch_actual_rad"], label="pitch actual")
    axes[0].set_ylabel("position [rad]")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(time_s, data["yaw_error_rad"], label="yaw error")
    axes[1].plot(time_s, data["pitch_error_rad"], label="pitch error")
    axes[1].set_ylabel("error [rad]")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(time_s, data["yaw_output"], label="yaw output")
    axes[2].plot(time_s, data["pitch_output"], label="pitch output")
    axes[2].set_ylabel("controller output")
    axes[2].grid(True)
    axes[2].legend()

    axes[3].plot(time_s, data["yaw_pwm"], label="yaw pwm")
    axes[3].plot(time_s, data["pitch_pwm"], label="pitch pwm")
    axes[3].plot(time_s, data["lateness_us"], label="lateness us")
    axes[3].set_ylabel("pwm / us")
    axes[3].set_xlabel("time [s]")
    axes[3].grid(True)
    axes[3].legend()

    fig.suptitle(path)
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
