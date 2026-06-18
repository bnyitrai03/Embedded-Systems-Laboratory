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
        if not rows or name not in rows[0]:
            return None
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
        "work_us": column("work_us"),
        "lateness_us": column("lateness_us"),
        "spi_exchange_us": column("spi_exchange_us"),
        "control_compute_us": column("control_compute_us"),
        "est_ball_yaw": column("est_ball_yaw"),
        "est_ball_pitch": column("est_ball_pitch"),
        "ramped_yaw": column("ramped_yaw"),
        "ramped_pitch": column("ramped_pitch"),
        "ball_vel_yaw": column("ball_vel_yaw"),
        "ball_vel_pitch": column("ball_vel_pitch"),
        "captured_age_ms": column("captured_age_ms"),
        "vision_lost": column("vision_lost"),
    }


def maybe_plot(axis, time_s, values, label):
    if values is not None:
        axis.plot(time_s, values, label=label)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "pid_log.csv"
    data = read_log(path)
    time_s = data["time_s"]

    position_fig, position_axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))

    maybe_plot(position_axes[0], time_s, data["est_ball_yaw"], "ball world (est)")
    maybe_plot(position_axes[0], time_s, data["ramped_yaw"], "ramped target")
    position_axes[0].plot(time_s, data["yaw_target_rad"], label="target")
    position_axes[0].plot(time_s, data["yaw_actual_rad"], label="actual")
    position_axes[0].set_ylabel("yaw [rad]")
    position_axes[0].grid(True)
    position_axes[0].legend()

    maybe_plot(position_axes[1], time_s, data["est_ball_pitch"], "ball world (est)")
    maybe_plot(position_axes[1], time_s, data["ramped_pitch"], "ramped target")
    position_axes[1].plot(time_s, data["pitch_target_rad"], label="target")
    position_axes[1].plot(time_s, data["pitch_actual_rad"], label="actual")
    position_axes[1].set_ylabel("pitch [rad]")
    position_axes[1].grid(True)
    position_axes[1].legend()

    position_axes[2].plot(time_s, data["yaw_error_rad"], label="yaw error")
    position_axes[2].plot(time_s, data["pitch_error_rad"], label="pitch error")
    maybe_plot(position_axes[2], time_s, data["ball_vel_yaw"], "ball vel yaw")
    maybe_plot(position_axes[2], time_s, data["ball_vel_pitch"], "ball vel pitch")
    position_axes[2].set_ylabel("error [rad] / vel [rad/s]")
    position_axes[2].set_xlabel("time [s]")
    position_axes[2].grid(True)
    position_axes[2].legend()

    position_fig.suptitle(f"{path} - position and error")
    position_fig.tight_layout()

    output_fig, output_axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))

    output_axes[0].plot(time_s, data["yaw_output"], label="yaw output")
    output_axes[0].plot(time_s, data["pitch_output"], label="pitch output")
    output_axes[0].set_ylabel("controller output")
    output_axes[0].grid(True)
    output_axes[0].legend()

    output_axes[1].plot(time_s, data["yaw_pwm"], label="yaw pwm")
    output_axes[1].plot(time_s, data["pitch_pwm"], label="pitch pwm")
    output_axes[1].plot(time_s, data["lateness_us"], label="lateness us")
    output_axes[1].set_ylabel("pwm / us")
    output_axes[1].grid(True)
    output_axes[1].legend()

    output_axes[2].plot(time_s, data["spi_exchange_us"], label="spi us")
    output_axes[2].plot(time_s, data["control_compute_us"], label="ctrl us")
    output_axes[2].plot(time_s, data["work_us"], label="work us")
    maybe_plot(output_axes[2], time_s, data["captured_age_ms"], "captured age ms")
    output_axes[2].set_ylabel("timing [us / ms]")
    output_axes[2].set_xlabel("time [s]")
    output_axes[2].grid(True)
    output_axes[2].legend()

    output_fig.suptitle(f"{path} - output and timing")
    output_fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
