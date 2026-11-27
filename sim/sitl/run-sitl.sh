#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ARDUPILOT_DIR="$ROOT_DIR/firmware/ardupilot"
PARM_FILE="$ROOT_DIR/sim/sitl/f450-rev-k.parm"
SITL_BIN="$ARDUPILOT_DIR/build/sitl/bin/arducopter"

if [[ -d "$ROOT_DIR/.venv/bin" ]]; then
  export PATH="$ROOT_DIR/.venv/bin:$PATH"
fi

if [[ ! -x "$SITL_BIN" ]]; then
  echo "BLOCKED: ArduCopter SITL binary not built yet ($SITL_BIN)." >&2
  echo "Build once host C++ toolchain is active (on macOS: 'sudo xcodebuild -license accept'):" >&2
  echo "  cd firmware/ardupilot && ./waf configure --board sitl --disable-scripting && ./waf copter" >&2
  exit 2
fi

echo "Starting ArduCopter SITL (F450 Rev K) — GCS udp:127.0.0.1:14550, Vision udp:127.0.0.1:14551"
exec python3 "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" \
  -v ArduCopter \
  -f quad \
  --no-rebuild \
  --add-param-file="$PARM_FILE" \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551
