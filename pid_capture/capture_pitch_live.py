from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parent
JLINK = Path(r"C:\Program Files\SEGGER\JLink_V982\JLink.exe")
LOG_PATH = ROOT / "20261116_pitch_live.log"
DONE_PATH = ROOT / "20261116_pitch_live.done"

commands = ["connect"]
for _ in range(2000):
    commands.extend(
        (
            "mem32 0x200008F8 1",
            "mem32 0x20000944 10",
            "Sleep 50",
        )
    )
commands.append("q")

with LOG_PATH.open("w", encoding="utf-8", errors="replace") as log:
    result = subprocess.run(
        [
            str(JLINK),
            "-device", "STM32F407IG",
            "-if", "SWD",
            "-speed", "1000",
            "-autoconnect", "0",
        ],
        input="\n".join(commands) + "\n",
        text=True,
        stdout=log,
        stderr=subprocess.STDOUT,
        check=False,
    )

DONE_PATH.write_text(str(result.returncode), encoding="ascii")
