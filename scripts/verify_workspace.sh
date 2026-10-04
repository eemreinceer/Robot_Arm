#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:---quick}"

case "$mode" in
  --quick|--ci|--ros|--full) ;;
  *)
    echo "usage: $0 [--quick|--ci|--ros|--full]" >&2
    exit 64
    ;;
esac

cd "$repo_root"

git_metadata_available=false
if git -C "$repo_root" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git_metadata_available=true
fi

list_source_files() {
  local pattern="$1"
  if [[ "$git_metadata_available" == true ]]; then
    git -C "$repo_root" ls-files "$pattern"
  else
    find "$repo_root" \
      \( -path "$repo_root/.git" -o -path "$repo_root/build" \
         -o -path "$repo_root/install" -o -path "$repo_root/log" \
         -o -path "$repo_root/.venv-ci" \) -prune -o \
      -type f -name "$pattern" -printf '%P\n' | LC_ALL=C sort
  fi
}

required_files=(
  README.md
  CONTRIBUTING.md
  SECURITY.md
  docs/supported_versions.md
  docs/repository_artifact_inventory.md
  src/README.md
)

for path in "${required_files[@]}"; do
  if [[ ! -f "$path" ]]; then
    echo "required workspace file is missing: $path" >&2
    exit 1
  fi
done

while IFS= read -r shell_file; do
  bash -n "$shell_file"
done < <(list_source_files '*.sh')
echo "bash syntax PASS"

shellcheck_files=(
  launch_phase3_test.sh
  launch_phase4_test.sh
  scripts/verify_commit_identity.sh
  scripts/verify_workspace.sh
)

if command -v shellcheck >/dev/null 2>&1; then
  shellcheck -x "${shellcheck_files[@]}"
  echo "ShellCheck PASS"
elif [[ "$mode" == "--ci" ]]; then
  echo "shellcheck is required in --ci mode" >&2
  exit 1
else
  echo "ShellCheck SKIP (shellcheck is not installed)"
fi

python3 - "$repo_root" "$git_metadata_available" <<'PY'
import pathlib
import subprocess
import sys

root = pathlib.Path(sys.argv[1])
if sys.argv[2] == "true":
    files = subprocess.run(
        ["git", "ls-files", "*.py"],
        cwd=str(root),
        check=True,
        text=True,
        capture_output=True,
    ).stdout.splitlines()
else:
    excluded_roots = {".git", ".venv-ci", "build", "install", "log"}
    files = sorted(
        str(path.relative_to(root))
        for path in root.rglob("*.py")
        if path.relative_to(root).parts[0] not in excluded_roots
    )

for relative in files:
    source = (root / relative).read_text(encoding="utf-8")
    compile(source, relative, "exec")

print("Python syntax PASS ({} files)".format(len(files)))
PY

python3 scripts/check_markdown_links.py \
  README.md \
  CONTRIBUTING.md \
  SECURITY.md \
  docs/supported_versions.md \
  docs/repository_artifact_inventory.md \
  src/README.md

if grep -nE '(^|[[:space:]])(ros2 launch|nohup)([[:space:]]|$)' \
  launch_phase3_test.sh launch_phase4_test.sh; then
  echo "retired entry points still contain forbidden launch commands" >&2
  exit 1
fi

if grep -nE 'ROBOT[K]OL[/]|[/]home[/][^/]+[/]Robot_Arm' \
  launch_phase3_test.sh launch_phase4_test.sh; then
  echo "retired entry points still contain the stale absolute workspace path" >&2
  exit 1
fi

if [[ "$git_metadata_available" == true ]]; then
  legacy_repo_pattern='6DOF[_]Robotic[_]Arm'
  if git -C "$repo_root" grep -nE "$legacy_repo_pattern" -- .; then
    echo "stale pre-portfolio repository identifier is still tracked" >&2
    exit 1
  fi

  internal_tool_pattern='\.clau''de[/]|\.co''dex[/]|\.ag''ents[/]|co''dex|skill[[:space:]]+gotcha'
  if git -C "$repo_root" grep -niE "$internal_tool_pattern" -- .; then
    echo "internal development-tool metadata is still tracked" >&2
    exit 1
  fi

  internal_path_pattern='(^|/)(AGENTS\.md|\.clau''de|\.co''dex|\.ag''ents)(/|$)'
  if git -C "$repo_root" ls-files | grep -Ei "$internal_path_pattern"; then
    echo "internal agent or coordination files must not be tracked" >&2
    exit 1
  fi

  if git -C "$repo_root" ls-files '*.pt' '*.pth' '*.onnx' | grep -q .; then
    echo "generated model weights must not be tracked in the public repository" >&2
    exit 1
  fi
fi

if [[ "$git_metadata_available" == true ]]; then
  git -C "$repo_root" diff --check
else
  echo "git metadata unavailable; working-tree diff check SKIP"
fi
echo "repository policy checks PASS"

if [[ "$mode" == "--ros" || "$mode" == "--full" ]]; then
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy is required for $mode" >&2
    exit 1
  fi
  if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon is required for $mode" >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
  verify_root="$(mktemp -d "${TMPDIR:-/tmp}/robot-arm-verify.XXXXXX")"
  cleanup_verify_root() {
    local status=$?
    if [[ "$status" -eq 0 ]]; then
      rm -rf -- "$verify_root"
    else
      echo "ROS verification artifacts preserved at: $verify_root" >&2
    fi
    return "$status"
  }
  trap cleanup_verify_root EXIT

  colcon --log-base "$verify_root/log" build \
    --build-base "$verify_root/build" \
    --install-base "$verify_root/install" \
    --symlink-install
  set +u
  # shellcheck disable=SC1091
  source "$verify_root/install/setup.bash"
  set -u
  colcon --log-base "$verify_root/test-log" test \
    --build-base "$verify_root/build" \
    --install-base "$verify_root/install"
  colcon test-result --test-result-base "$verify_root/build" --verbose
  echo "clean ROS 2 workspace verification PASS"
fi

if [[ "$mode" == "--full" ]]; then
  if ! command -v pio >/dev/null 2>&1; then
    echo "PlatformIO is required for --full" >&2
    exit 1
  fi
  pio test -d firmware/stm32_servo_ctrl -e native
  pio test -d firmware/esp32_servo_ctrl -e native
  echo "native firmware verification PASS"
fi

echo "workspace verification PASS ($mode)"
