#!/usr/bin/env bash
# One-step setup for mujoco-puzzle-gym (macOS and Linux).
#   ./install.sh
# 1. installs uv if missing (official installer from astral.sh),
# 2. installs Python 3.12 and the dependencies into .venv (uv sync),
# 3. downloads the pinned Microduck model and walking policy and checks their SHA256,
# 4. checks the example room to confirm everything works.
set -euo pipefail
cd "$(dirname "$0")"

say() { printf '\n\033[1m==> %s\033[0m\n' "$*"; }

if ! command -v uv >/dev/null 2>&1; then
  if [ -x "$HOME/.local/bin/uv" ]; then
    export PATH="$HOME/.local/bin:$PATH"
  else
    say "uv (Python package manager) not found: installing it from https://astral.sh/uv"
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
  fi
fi

say "Installing Python 3.12 and dependencies (uv sync)"
uv sync

say "Downloading the Microduck model and walking policy (about 22 MB)"
uv run mpg fetch-assets

say "Checking the example room"
uv run mpg check examples/switch_door.yaml

cat <<'MSG'

Done. Try:
  uv run mpg demo                                    # trained duck solves the example room -> demo.mp4
  uv run mpg train examples/task_ppo.yaml --minutes 15 --id first
  uv run mpg render runs/first                       # video of your own policy
MSG
