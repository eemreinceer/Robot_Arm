"""PlatformIO pre-build guard for the checked-in generated calibration."""

from pathlib import Path
import subprocess
import sys

Import("env")  # type: ignore[name-defined]  # noqa: F821

project_dir = Path(env.subst("$PROJECT_DIR"))  # type: ignore[name-defined]  # noqa: F821
system_python = Path("/usr/bin/python3")
python_executable = str(system_python) if system_python.is_file() else sys.executable
subprocess.check_call(
    [
        python_executable,
        str(project_dir / "tools/generate_robot_config.py"),
        "--check",
    ],
    cwd=project_dir,
)
