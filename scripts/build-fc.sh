#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_DIR="$ROOT_DIR/firmware/ardupilot"
BOARD_SRC="$ROOT_DIR/firmware/board-openfcf405"
BOARD_NAME="OPENFCF405"
BOARD_DST="$ARDUPILOT_DIR/libraries/AP_HAL_ChibiOS/hwdef/$BOARD_NAME"
REPORT_DIR="$ROOT_DIR/reports"
STAMP="$(date -u +%Y%m%d-%H%M%SZ)"
LOG_FILE="$REPORT_DIR/build-openfcf405-$STAMP.log"

# STM32F405RGT6 1024 KB Flash layout:
#   Sectors 0-1 (32 KB): Bootloader (0x08000000 - 0x08007FFF)
#   Sectors 2-3 (32 KB): Dual-page AP_FlashStorage (0x08008000 - 0x0800FFFF)
#   Sectors 4-11 (960 KB): Main ArduCopter firmware (0x08010000 - 0x080FFFFF)
BL_FLASH_LIMIT_BYTES=$((32 * 1024))
APP_FLASH_LIMIT_BYTES=$((960 * 1024))

if [[ -d "$ROOT_DIR/.venv/bin" ]]; then
  export PATH="$ROOT_DIR/.venv/bin:$PATH"
fi

# Prefer repo-local official Arm GNU Toolchain (includes newlib headers/libc) over bare Homebrew gcc
for tc_bin in "$ROOT_DIR"/.toolchain/*/bin; do
  if [[ -x "$tc_bin/arm-none-eabi-gcc" ]]; then
    export PATH="$tc_bin:$PATH"
    break
  fi
done

if [[ ! -x "$ARDUPILOT_DIR/waf" ]]; then
  echo "BLOCKED: firmware/ardupilot submodule is not populated ($ARDUPILOT_DIR/waf missing)." >&2
  exit 2
fi

for tool in arm-none-eabi-gcc arm-none-eabi-size arm-none-eabi-nm; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "BLOCKED: $tool not found in PATH." >&2
    exit 2
  fi
done

TEMP_BIN=""
CLEANED=0

ensure_temp_bin() {
  if [[ -z "$TEMP_BIN" ]]; then
    TEMP_BIN="$(mktemp -d)"
    export PATH="$TEMP_BIN:$PATH"
  fi
}

cleanup() {
  if [[ "$CLEANED" -eq 1 ]]; then
    return
  fi
  CLEANED=1
  rm -rf "$BOARD_DST" "$ARDUPILOT_DIR/Tools/bootloaders/${BOARD_NAME}_bl."*
  if [[ -n "$TEMP_BIN" ]]; then
    rm -rf "$TEMP_BIN"
  fi
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

if ! make --version >/dev/null 2>&1; then
  echo "INFO: host 'make' unavailable; using scripts/chibios_make.py fallback for ChibiOS."
  ensure_temp_bin
  ln -s "$ROOT_DIR/scripts/chibios_make.py" "$TEMP_BIN/make"
fi

if ! gcc --version >/dev/null 2>&1; then
  echo "INFO: host 'gcc' unavailable; scripting is disabled (--disable-scripting), installing no-op gen-bindings stub."
  ensure_temp_bin
  cat > "$TEMP_BIN/gcc" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
out=""
is_gen_bindings=0
prev=""
for arg in "$@"; do
  if [[ "$prev" == "-o" ]]; then
    out="$arg"
  fi
  if [[ "$arg" == *"AP_Scripting/generator/src/main.c"* ]]; then
    is_gen_bindings=1
  fi
  prev="$arg"
done
if [[ "$is_gen_bindings" -eq 1 && -n "$out" ]]; then
  cat > "$out" <<'GEN'
#!/usr/bin/env bash
set -euo pipefail
prefix=""
prev=""
for a in "$@"; do
  [[ "$prev" == "-o" ]] && prefix="$a"
  prev="$a"
done
mkdir -p "$(dirname "$prefix")"
printf '/* AP_SCRIPTING_ENABLED=0: empty bindings stub */\n' > "${prefix}.h"
printf '/* AP_SCRIPTING_ENABLED=0: empty bindings stub */\n' > "${prefix}.cpp"
GEN
  chmod +x "$out"
  exit 0
fi
echo "ERROR: host gcc is unavailable and unexpected host compilation was requested: $*" >&2
exit 2
EOF
  chmod +x "$TEMP_BIN/gcc"
fi

mkdir -p "$REPORT_DIR"

rm -rf "$BOARD_DST" "$ARDUPILOT_DIR/Tools/bootloaders/${BOARD_NAME}_bl."*
cp -R "$BOARD_SRC" "$BOARD_DST"

echo "Building $BOARD_NAME bootloader + ArduCopter (log: $LOG_FILE)..."
(
  cd "$ARDUPILOT_DIR"
  echo "--- Step 1/2: Bootloader ($BOARD_NAME) ---"
  rm -rf "build/$BOARD_NAME/modules/ChibiOS" "build/$BOARD_NAME/.dep"
  ./waf configure --board "$BOARD_NAME" --bootloader --disable-scripting --no-submodule-update
  ./waf bootloader
  BL_TMP="build/$BOARD_NAME/bin/AP_Bootloader.bin"
  if [[ ! -f "$BL_TMP" ]]; then
    echo "FAIL: expected bootloader binary not found at $BL_TMP" >&2
    exit 1
  fi
  BL_SZ="$(wc -c < "$BL_TMP" | tr -d ' ')"
  if (( BL_SZ > BL_FLASH_LIMIT_BYTES )); then
    echo "FAIL: bootloader (${BL_SZ} B) exceeds 32 KB slot (${BL_FLASH_LIMIT_BYTES} B, Sectors 0-1)." >&2
    exit 1
  fi
  cp "$BL_TMP" "Tools/bootloaders/${BOARD_NAME}_bl.bin"

  echo "--- Step 2/2: ArduCopter ($BOARD_NAME) ---"
  # Clear bootloader ChibiOS objects so HAL_BOOTLOADER_BUILD objects can never leak into app libch.a
  rm -rf "build/$BOARD_NAME/modules/ChibiOS" "build/$BOARD_NAME/.dep"
  ./waf configure --board "$BOARD_NAME" --disable-scripting --no-submodule-update
  ./waf copter
) 2>&1 | tee "$LOG_FILE"

BL_BIN_FILE="$ARDUPILOT_DIR/build/$BOARD_NAME/bin/AP_Bootloader.bin"
BL_ELF_FILE="$ARDUPILOT_DIR/build/$BOARD_NAME/bootloader/AP_Bootloader"
if [[ ! -f "$BL_BIN_FILE" ]]; then
  echo "FAIL: expected bootloader binary not found at $BL_BIN_FILE" | tee -a "$LOG_FILE" >&2
  exit 1
fi

BL_BIN_SIZE="$(wc -c < "$BL_BIN_FILE" | tr -d ' ')"
BL_HEADROOM="$((BL_FLASH_LIMIT_BYTES - BL_BIN_SIZE))"
echo "Bootloader Flash usage: ${BL_BIN_SIZE} / ${BL_FLASH_LIMIT_BYTES} bytes (headroom: ${BL_HEADROOM} bytes)" | tee -a "$LOG_FILE"
if [[ -f "$BL_ELF_FILE" ]]; then
  arm-none-eabi-size "$BL_ELF_FILE" | tee -a "$LOG_FILE"
fi
if (( BL_BIN_SIZE > BL_FLASH_LIMIT_BYTES )); then
  echo "FAIL: bootloader exceeds 32 KB slot (Sectors 0-1) by $((-BL_HEADROOM)) bytes." | tee -a "$LOG_FILE" >&2
  exit 1
fi

BIN_FILE="$ARDUPILOT_DIR/build/$BOARD_NAME/bin/arducopter.bin"
ELF_FILE="$ARDUPILOT_DIR/build/$BOARD_NAME/bin/arducopter"
if [[ ! -f "$BIN_FILE" || ! -f "$ELF_FILE" ]]; then
  echo "FAIL: expected app artifacts missing ($BIN_FILE / $ELF_FILE)" | tee -a "$LOG_FILE" >&2
  exit 1
fi

BIN_SIZE="$(wc -c < "$BIN_FILE" | tr -d ' ')"
HEADROOM="$((APP_FLASH_LIMIT_BYTES - BIN_SIZE))"
echo "Flash usage: ${BIN_SIZE} / ${APP_FLASH_LIMIT_BYTES} bytes (headroom: ${HEADROOM} bytes)" | tee -a "$LOG_FILE"
arm-none-eabi-size "$ELF_FILE" | tee -a "$LOG_FILE"

if (( BIN_SIZE > APP_FLASH_LIMIT_BYTES )); then
  echo "FAIL: firmware exceeds 960 KB app slot (1 MB Flash ceiling) by $((-HEADROOM)) bytes." | tee -a "$LOG_FILE" >&2
  echo "Disable only non-baseline features; NEVER remove EKF, PreArm, failsafes, or PrecLand (AGENTS.md §1.3)." >&2
  exit 1
fi

HWDEF_H="$ARDUPILOT_DIR/build/$BOARD_NAME/hwdef.h"
if [[ ! -f "$HWDEF_H" ]]; then
  echo "FAIL: generated $HWDEF_H not found." | tee -a "$LOG_FILE" >&2
  exit 1
fi
python3 -B "$ROOT_DIR/scripts/check-dma.py" "$HWDEF_H" | tee -a "$LOG_FILE"

# Verify ROMFS embedding of defaults.parm and bootloader.bin
ROMFS_H="$ARDUPILOT_DIR/build/$BOARD_NAME/ap_romfs_embedded.h"
PROC_DEFAULTS="$ARDUPILOT_DIR/build/$BOARD_NAME/processed_defaults.parm"
if [[ ! -s "$PROC_DEFAULTS" ]]; then
  echo "FAIL: processed_defaults.parm missing or empty at $PROC_DEFAULTS" | tee -a "$LOG_FILE" >&2
  exit 1
fi
if ! grep -q '^#define AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED 1$' "$HWDEF_H"; then
  echo "FAIL: AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED is not enabled in $HWDEF_H" | tee -a "$LOG_FILE" >&2
  exit 1
fi
if [[ ! -f "$ROMFS_H" ]] || ! grep -q '"defaults.parm"' "$ROMFS_H" || ! grep -q '"bootloader.bin"' "$ROMFS_H"; then
  echo "FAIL: ROMFS missing defaults.parm or bootloader.bin in $ROMFS_H" | tee -a "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: ROMFS embeds defaults.parm ($(wc -c < "$PROC_DEFAULTS" | tr -d ' ') B) and bootloader.bin (${BL_BIN_SIZE} B)." | tee -a "$LOG_FILE"

echo "Checking mandatory safety & Rev K sensor symbols in ELF..." | tee -a "$LOG_FILE"
NM_OUT="$(arm-none-eabi-nm --defined-only -C "$ELF_FILE")"
for sym in \
  "NavEKF3::InitialiseFilter" \
  "AP_Arming::pre_arm_checks" \
  "failsafe_radio_on_event" \
  "handle_battery_failsafe" \
  "failsafe_ekf_event" \
  "AC_PrecLand_Companion" \
  "AP_InertialSensor_Invensense" \
  "AP_Baro_BMP280" \
  "AP_GPS_UBLOX" \
  "AP_RCProtocol_SBUS" \
  "send_pulses_DMAR"; do
  if ! grep -q "$sym" <<< "$NM_OUT"; then
    echo "FAIL: mandatory symbol missing from ELF: $sym (AGENTS.md §1.3)" | tee -a "$LOG_FILE" >&2
    exit 1
  fi
done
echo "PASS: all 11 mandatory safety/sensor/PrecLand/DMAR symbols verified in ELF." | tee -a "$LOG_FILE"

for sym in \
  "AP_Compass_HMC5843" \
  "AP_Compass_QMC5883L" \
  "AP_Compass_QMC5883P" \
  "AP_Param::reload_defaults_file" \
  "AP_ROMFS::find_decompress"; do
  if ! grep -q "$sym" <<< "$NM_OUT"; then
    echo "FAIL: expected Rev K compass/ROMFS symbol missing from ELF: $sym" | tee -a "$LOG_FILE" >&2
    exit 1
  fi
done
echo "PASS: BN-880 compass backends (HMC5843/QMC5883L/QMC5883P) and ROMFS defaults loader verified in ELF." | tee -a "$LOG_FILE"

cleanup
SUBMOD_DIRTY="$(git -C "$ARDUPILOT_DIR" status --short)"
if [[ -n "$SUBMOD_DIRTY" ]]; then
  echo "FAIL: firmware/ardupilot submodule is not clean after build:" | tee -a "$LOG_FILE" >&2
  echo "$SUBMOD_DIRTY" | tee -a "$LOG_FILE" >&2
  exit 1
fi

echo "PASS: $BOARD_NAME built within 1 MB Flash ceiling (submodule clean)." | tee -a "$LOG_FILE"
