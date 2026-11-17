import csv
import re
import struct
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
YAW_CSV = ROOT / "20261116_yaw_response.csv"
PITCH_LOG = ROOT / "20261116_pitch_live.log"
PITCH_CSV = ROOT / "20261116_pitch_response.csv"
PITCH_LIMIT_CSV = ROOT / "20261116_pitch_limit_verify.csv"
PITCH_TUNED_CSV = ROOT / "20261116_pitch_tuned_response.csv"
PITCH_KD020_CSV = ROOT / "20261116_pitch_kd020_response.csv"
YAW_KD010_CSV = ROOT / "20261116_yaw_kd010_response.csv"
YAW_KD020_CSV = ROOT / "20261116_yaw_kd020_response.csv"
YAW_FINAL_RIGHT_CSV = ROOT / "20261116_yaw_final_right_response.csv"
YAW_FINAL_LEFT_CSV = ROOT / "20261116_yaw_final_left_response.csv"


def raw_float(word):
    return struct.unpack(">f", struct.pack(">I", word))[0]


def parse_pitch_log():
    rows = []
    current = None
    tick_pattern = re.compile(r"200008F8\s*=\s*([0-9A-Fa-f]{8})")
    data_pattern = re.compile(r"200009(?:44|54|64)\s*=\s*(.*)")
    word_pattern = re.compile(r"[0-9A-Fa-f]{8}")

    for line in PITCH_LOG.read_text(encoding="utf-8", errors="replace").splitlines():
        tick_match = tick_pattern.search(line)
        if tick_match:
            if current and len(current["words"]) >= 10:
                rows.append(current)
            current = {"tick": int(tick_match.group(1), 16), "words": []}
            continue
        if current is None:
            continue
        data_match = data_pattern.search(line)
        if data_match:
            current["words"].extend(
                int(word, 16) for word in word_pattern.findall(data_match.group(1))
            )
    if current and len(current["words"]) >= 10:
        rows.append(current)

    if not rows:
        raise RuntimeError("No pitch samples found")

    tick0 = rows[0]["tick"]
    parsed = []
    for row in rows:
        words = row["words"]
        parsed.append(
            {
                "time_s": ((row["tick"] - tick0) & 0xFFFFFFFF) / 1000.0,
                "tick_ms": row["tick"],
                "remote_rv": raw_float(words[1]),
                "joint_pitch": raw_float(words[3]),
                "target_pitch": raw_float(words[5]),
                "pitch_error": raw_float(words[7]),
                "pitch_output": raw_float(words[9]),
            }
        )

    with PITCH_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=parsed[0].keys())
        writer.writeheader()
        writer.writerows(parsed)
    return parsed


def load_csv(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(handle)
        ]


def active_window(rows, input_key, pre=20, post=180):
    active = [index for index, row in enumerate(rows) if abs(row[input_key]) > 0.02]
    if not active:
        return rows
    return rows[max(0, active[0] - pre) : min(len(rows), active[-1] + post)]


def draw_response(rows, target, feedback, error, output, remote, title, path):
    time0 = rows[0]["time_s"]
    time = [row["time_s"] - time0 for row in rows]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    axes[0].plot(time, [row[target] for row in rows], "--", color="#111827", label="Target")
    axes[0].plot(time, [row[feedback] for row in rows], color="#16a34a", label="Feedback")
    axes[0].set_ylabel("Angle (rad)")
    axes[0].legend(loc="best")

    axes[1].plot(time, [row[error] for row in rows], color="#dc2626", label="Error")
    axes[1].axhline(0.0, color="#6b7280", linewidth=0.8)
    axes[1].set_ylabel("Error (rad)")
    axes[1].legend(loc="best")

    axes[2].plot(time, [row[remote] for row in rows], color="#2563eb", label="Remote")
    output_axis = axes[2].twinx()
    output_axis.plot(time, [row[output] for row in rows], color="#9333ea", label="PID output")
    axes[2].set_ylabel("Remote")
    output_axis.set_ylabel("Output")
    axes[2].set_xlabel("Time (s)")
    lines = axes[2].lines + output_axis.lines
    axes[2].legend(lines, [line.get_label() for line in lines], loc="best")

    for axis in axes:
        axis.grid(True, alpha=0.25)
    fig.suptitle(title + "\nManual multi-pulse input; not a laboratory fixed step")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=180)
    plt.close(fig)


yaw_rows = load_csv(YAW_CSV)
pitch_rows = parse_pitch_log()
pitch_plot_rows = active_window(pitch_rows, "remote_rv")

draw_response(
    yaw_rows,
    "target_yaw",
    "imu_yaw",
    "yaw_error",
    "yaw_output",
    "remote_rh",
    "Gimbal yaw PID response",
    ROOT / "20261116_yaw_response.png",
)
draw_response(
    pitch_plot_rows,
    "target_pitch",
    "joint_pitch",
    "pitch_error",
    "pitch_output",
    "remote_rv",
    "Gimbal pitch PID response",
    ROOT / "20261116_pitch_response.png",
)
if PITCH_LIMIT_CSV.exists():
    draw_response(
        load_csv(PITCH_LIMIT_CSV),
        "target_pitch",
        "joint_pitch",
        "pitch_error",
        "pitch_output",
        "remote_rv",
        "Gimbal pitch lower-limit verification",
        ROOT / "20261116_pitch_limit_verify.png",
    )
if PITCH_TUNED_CSV.exists():
    draw_response(
        load_csv(PITCH_TUNED_CSV),
        "target_pitch",
        "joint_pitch",
        "pitch_error",
        "pitch_output",
        "remote_rv",
        "Gimbal pitch tuned PID response",
        ROOT / "20261116_pitch_tuned_response.png",
    )
if PITCH_KD020_CSV.exists():
    draw_response(
        load_csv(PITCH_KD020_CSV),
        "target_pitch",
        "joint_pitch",
        "pitch_error",
        "pitch_output",
        "remote_rv",
        "Gimbal pitch PID response (Kd=0.20)",
        ROOT / "20261116_pitch_kd020_response.png",
    )
if YAW_KD010_CSV.exists():
    draw_response(
        load_csv(YAW_KD010_CSV),
        "target_yaw",
        "imu_yaw",
        "yaw_error",
        "yaw_output",
        "remote_rh",
        "Gimbal yaw PID response (Kd=0.10)",
        ROOT / "20261116_yaw_kd010_response.png",
    )
if YAW_KD020_CSV.exists():
    draw_response(
        load_csv(YAW_KD020_CSV),
        "target_yaw",
        "imu_yaw",
        "yaw_error",
        "yaw_output",
        "remote_rh",
        "Gimbal yaw PID response (Kd=0.20)",
        ROOT / "20261116_yaw_kd020_response.png",
    )
for final_csv, final_name, final_title in (
    (YAW_FINAL_RIGHT_CSV, "20261116_yaw_final_right_response.png", "Gimbal yaw final right response"),
    (YAW_FINAL_LEFT_CSV, "20261116_yaw_final_left_response.png", "Gimbal yaw final left response"),
):
    if final_csv.exists():
        draw_response(
            load_csv(final_csv),
            "target_yaw",
            "imu_yaw",
            "yaw_error",
            "yaw_output",
            "remote_rh",
            final_title,
            ROOT / final_name,
        )

for name, rows, error_key in (
    ("yaw", yaw_rows, "yaw_error"),
    ("pitch", pitch_rows, "pitch_error"),
):
    errors = [row[error_key] for row in rows]
    tail = errors[-20:]
    print(
        name,
        "samples=", len(rows),
        "max_abs_error=", f"{max(abs(value) for value in errors):.6f}",
        "tail_mean_error=", f"{sum(tail) / len(tail):.6f}",
    )
