import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
CSV_PATH = ROOT / "20261116_follow_response.csv"
PNG_PATH = ROOT / "20261116_follow_response.png"


with CSV_PATH.open(newline="", encoding="utf-8") as handle:
    rows = list(csv.DictReader(handle))

time_s = [(float(row["t_ms"]) - float(rows[0]["t_ms"])) / 1000.0 for row in rows]


def values(name):
    return [float(row[name]) for row in rows]


fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

axes[0].plot(time_s, values("gimbal_yaw"), label="Relative gimbal yaw", color="#2563eb", linewidth=1.8)
axes[0].plot(time_s, values("follow_error"), label="Follow error", color="#dc2626", linestyle="--", linewidth=1.4)
axes[0].set_ylabel("rad")
axes[0].set_title("Follow-loop input while chassis body is fixed off ground")
axes[0].legend(loc="best")

axes[1].plot(time_s, values("applied_w"), label="Applied yaw-rate command", color="#7c3aed", linewidth=1.8)
axes[1].plot(time_s, values("follow_output"), label="Follow controller output", color="#ea580c", linestyle="--", linewidth=1.4)
axes[1].axhline(-3.0, color="#111827", linestyle=":", linewidth=1.0, label="Configured limit")
axes[1].set_ylabel("rad/s")
axes[1].set_title("Bounded command at the configured saturation limit")
axes[1].legend(loc="best")

colors = {"lf": "#2563eb", "lb": "#16a34a", "rb": "#dc2626", "rf": "#ca8a04"}
for key, label in (("lf", "LF"), ("lb", "LB"), ("rb", "RB"), ("rf", "RF")):
    axes[2].plot(time_s, values(f"target_{key}"), color=colors[key], linestyle="--", linewidth=1.1)
    axes[2].plot(time_s, values(f"speed_{key}"), label=f"{label} feedback", color=colors[key], linewidth=1.6)
axes[2].set_ylabel("rad/s")
axes[2].set_xlabel("Time (s)")
axes[2].set_title("Wheel targets and feedback during follow saturation")
axes[2].legend(loc="best", ncol=2)

for axis in axes:
    axis.grid(True, alpha=0.25)

fig.text(
    0.5,
    0.012,
    "Bench condition: wheels off ground and chassis body fixed. This capture proves command limiting and wheel tracking; "
    "it is not evidence of follow-angle settling.",
    ha="center",
    fontsize=9,
)
fig.suptitle("Chassis follow saturation and wheel-speed response (2026-11-16)", fontsize=15)
fig.tight_layout(rect=(0, 0.035, 1, 0.965))
fig.savefig(PNG_PATH, dpi=180)
print(PNG_PATH)
