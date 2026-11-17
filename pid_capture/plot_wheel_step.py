import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
CSV_PATH = ROOT / "20261115_wheel_step.csv"
PNG_PATH = ROOT / "20261115_wheel_step.png"


with CSV_PATH.open(newline="", encoding="utf-8") as handle:
    rows = list(csv.DictReader(handle))

time_s = [float(row["time_s"]) for row in rows]
remote = [float(row["remote_vx"]) for row in rows]
applied = [float(row["applied_vx"]) for row in rows]

fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
axes = axes.ravel()

axes[0].plot(time_s, remote, label="Remote vx", color="#2563eb", linewidth=1.8)
axes[0].plot(time_s, applied, label="Applied vx", color="#dc2626", linewidth=1.8)
axes[0].set_title("Input and acceleration-limited command")
axes[0].set_ylabel("Normalized / m/s")
axes[0].legend(loc="best")

wheels = [("lf", "LF"), ("lb", "LB"), ("rb", "RB"), ("rf", "RF")]
for axis, (key, label) in zip(axes[1:5], wheels):
    target = [float(row[f"{key}_target"]) for row in rows]
    speed = [float(row[f"{key}_speed"]) for row in rows]
    axis.plot(time_s, target, label="Target", color="#111827", linestyle="--", linewidth=1.8)
    axis.plot(time_s, speed, label="Feedback", color="#16a34a", linewidth=1.8)
    axis.set_title(f"{label} wheel speed loop")
    axis.set_ylabel("rad/s")
    axis.legend(loc="best")

for axis in axes[:5]:
    axis.axvline(1.157, color="#9333ea", linestyle=":", linewidth=1.2)
    axis.grid(True, alpha=0.25)
    axis.set_xlabel("Time (s)")

axes[5].axis("off")
axes[5].text(
    0.02,
    0.92,
    "Capture: 2026-11-15\n"
    "Firmware stage: Full\n"
    "Sampling: J-Link, about 100 ms\n"
    "Step detected: 1.157 s\n"
    "Bench condition: wheels off ground\n\n"
    "Use this plot as field evidence.\n"
    "The manual stick target varies slightly,\n"
    "so it is not a laboratory-grade fixed step.",
    va="top",
    fontsize=11,
)

fig.suptitle("Chassis wheel PID step response", fontsize=16)
fig.tight_layout(rect=(0, 0, 1, 0.97))
fig.savefig(PNG_PATH, dpi=180)
print(PNG_PATH)
