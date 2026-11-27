#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KICAD_DIR="$ROOT_DIR/hardware/kicad/open-fc-carrier-rev-k"
SCH_FILE="$KICAD_DIR/open-fc-carrier.kicad_sch"
PCB_FILE="$KICAD_DIR/open-fc-carrier.kicad_pcb"
FAB_DIR="$ROOT_DIR/hardware/fabrication"
REPORT_DIR="$ROOT_DIR/reports"
STAMP="$(date -u +%Y%m%d-%H%M%SZ)"

if ! command -v kicad-cli >/dev/null 2>&1; then
  echo "BLOCKED: kicad-cli not found in PATH." >&2
  exit 2
fi

if [[ ! -f "$SCH_FILE" || ! -f "$PCB_FILE" ]]; then
  echo "BLOCKED: KiCad Rev K schematic/PCB not present yet ($SCH_FILE / $PCB_FILE)." >&2
  echo "Place Sheet 1-3 reference PNGs in hardware/reference-schematics/ and build via KiCad / Konnect MCP." >&2
  exit 2
fi

mkdir -p "$REPORT_DIR" "$FAB_DIR"

ERC_JSON="$REPORT_DIR/erc-rev-k-$STAMP.json"
DRC_JSON="$REPORT_DIR/drc-rev-k-$STAMP.json"
BOM_CSV="$FAB_DIR/kicad-bom-rev-k.csv"
FIT_PDF="$FAB_DIR/open-fc-carrier-rev-k-1to1-A4.pdf"
SCH_PDF="$FAB_DIR/open-fc-carrier-rev-k-schematic.pdf"

echo "[1/5] Running KiCad ERC (0-violation gate)..."
kicad-cli sch erc --format json --units mm --severity-all --exit-code-violations --output "$ERC_JSON" "$SCH_FILE"

echo "[2/5] Running KiCad DRC + schematic parity (0-violation gate)..."
kicad-cli pcb drc --format json --units mm --schematic-parity --severity-all --exit-code-violations --output "$DRC_JSON" "$PCB_FILE"

echo "[3/5] Exporting 1:1 scale A4 mechanical fit-check PDF..."
kicad-cli pcb export pdf \
  --output "$FIT_PDF" \
  --layers "F.Cu,B.Cu,F.SilkS,F.Fab,Edge.Cuts" \
  --mode-single \
  --scale 1 \
  --drill-shape-opt 2 \
  --include-border-title \
  "$PCB_FILE"

echo "[4/5] Exporting schematic PDF & BOM CSV..."
kicad-cli sch export pdf --output "$SCH_PDF" "$SCH_FILE"
kicad-cli sch export bom --output "$BOM_CSV" "$SCH_FILE"

echo "[5/5] Checking BOM synchronization against docs/bom-rev-k.csv..."
python3 "$ROOT_DIR/scripts/check-bom-sync.py" --kicad-bom "$BOM_CSV"

echo "PASS: KiCad Rev K ERC=0, DRC=0, 1:1 PDF ($FIT_PDF), and BOM sync complete."
