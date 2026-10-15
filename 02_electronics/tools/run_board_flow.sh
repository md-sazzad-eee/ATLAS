#!/bin/bash
# Full board flow from a design spec:  spec -> schematic -> netlist check -> (ERC) -> placement ->
# Freerouting -> GND pours/stitching -> completion passes -> DRC -> fab outputs.
#   tools/run_board_flow.sh dock_charger       (Board C)
#   tools/run_board_flow.sh carrier_controller (Board A)
# Works on macOS (KiCad 10 app bundle) and Linux (KiCad 7+ packages). Routing needs Java 21 + Freerouting 2.x
# (FR=/path/freerouting.jar); without it the flow stops after placement and says so.
set -e
TOOLS="$(cd "$(dirname "$0")" && pwd)"
BOARD_DIR="$(cd "$TOOLS/../$1" && pwd)"
CFG=$(ls "$BOARD_DIR"/design/board_*_pcb_cfg.py)
DESIGN=$(ls "$BOARD_DIR"/design/board_*_design.py)
NAME=$(python3 -c "import runpy;print(runpy.run_path('$CFG')['NAME'])")
TITLE=$(python3 -c "import runpy,re;d=open('$DESIGN').read();print(re.search(r'\"\"\"(ATLAS Board [A-Z][^\n—]*)',d).group(1).strip())")
if [ -d /Applications/KiCad/KiCad.app ]; then
  KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
  KCLI=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
  JAVA=${JAVA:-java}   # any Java 21+
  FR=${FR:-$HOME/tools/freerouting-2.1.0.jar}
else
  KPY=python3; KCLI=kicad-cli; JAVA=${JAVA:-java}; FR=${FR:-$HOME/tools/freerouting-2.1.0.jar}
fi
cd "$BOARD_DIR"
echo "== $TITLE ($NAME)"
# RESUME=1: keep the routed board on disk and go straight to the completion passes
# (used when a long run was interrupted, e.g. by the machine restarting)
if [ -z "$RESUME" ]; then
python3 "$TOOLS/check_design.py" "$DESIGN" | tail -1
python3 "$TOOLS/gen_schematic.py" "$DESIGN" "$NAME.kicad_sch" "$TITLE" A
"$KCLI" sch export netlist -o "$NAME.net" "$NAME.kicad_sch" > /dev/null
python3 "$TOOLS/verify_netlist.py" "$DESIGN" "$NAME.net"
"$KCLI" sch export pdf -o "${NAME}_schematic.pdf" "$NAME.kicad_sch" > /dev/null
if "$KCLI" sch erc --help > /dev/null 2>&1; then
  "$KCLI" sch erc --format json -o erc_report.json "$NAME.kicad_sch" > /dev/null || true
  python3 -c "import json;d=json.load(open('erc_report.json'));v=[x for s in d['sheets'] for x in s['violations']];print('ERC:',sum(x['severity']=='error' for x in v),'errors,',sum(x['severity']=='warning' for x in v),'warnings')"
fi
"$KPY" "$TOOLS/build_pcb.py" "$CFG" place 2>&1 | grep -v assert
if "$KCLI" pcb drc --help > /dev/null 2>&1; then          # placement must be clean before routing
  "$KCLI" pcb drc --format json -o drc_placement.json "$NAME.kicad_pcb" > /dev/null 2>&1 || true
  python3 -c "import json,collections;v=json.load(open('drc_placement.json'))['violations'];c=collections.Counter(x['type'] for x in v if not x['type'].startswith(('silk','text','lib_')));print('placement DRC (copper/edge):',dict(c) or 'clean')"
fi
fi
if [ ! -f "$FR" ]; then echo "Freerouting not found ($FR): stopping after placement"; exit 0; fi
route() {   # Freerouting never stops while a connection is unroutable: give it FR_TIMEOUT seconds
  rm -f "$2"; nice -n 10 "$JAVA" -Xmx4g -Djava.awt.headless=true -jar "$FR" -de "$1" -do "$2" --router.max_passes=${FR_PASSES:-100} -mt ${FR_THREADS:-4} \
      --gui.enabled=false --api_server.enabled=false > freerouting.log 2>&1 &
  local pid=$! t=0
  while kill -0 $pid 2>/dev/null; do sleep 5; t=$((t+5)); [ $t -ge ${FR_TIMEOUT:-1200} ] && { kill $pid; echo "Freerouting timed out after ${t}s: $(grep -o '([0-9]* unrouted[^)]*)' freerouting.log | tail -1)"; return 1; }; done
  [ -f "$2" ]; }
if [ -z "$RESUME" ]; then
route "$NAME.dsn" "$NAME.ses"
"$KPY" "$TOOLS/build_pcb.py" "$CFG" import_ses "$NAME.ses" 2>&1 | grep -v assert
"$KPY" "$TOOLS/build_pcb.py" "$CFG" bridge 2>&1 | grep -v assert
fi
for i in 1 2 3; do
  U=$("$KPY" "$TOOLS/build_pcb.py" "$CFG" unconnected 2>&1 | grep UNCONNECTED | awk '{print $2}')
  echo "unconnected after pass $i: $U"; [ "$U" = "0" ] && break
  "$KPY" "$TOOLS/build_pcb.py" "$CFG" export_routed "${NAME}_complete.dsn" 2>&1 | grep -v assert
  if FR_TIMEOUT=${FR_TIMEOUT_COMPLETE:-600} route "${NAME}_complete.dsn" "${NAME}_complete.ses"; then     # a timed-out pass keeps the board as is
    "$KPY" "$TOOLS/build_pcb.py" "$CFG" import_ses "${NAME}_complete.ses" 2>&1 | grep -v assert
    "$KPY" "$TOOLS/build_pcb.py" "$CFG" bridge 2>&1 | grep -v assert
  else
    break                                                          # same input again would time out again
  fi
done
# whatever Freerouting could not finish: deterministic grid router over all layers (needs numpy, NPY=python)
U=$("$KPY" "$TOOLS/build_pcb.py" "$CFG" unconnected 2>&1 | grep UNCONNECTED | awk '{print $2}')
NPY=${NPY:-python3}
if [ "$U" != "0" ] && "$NPY" -c "import numpy" 2>/dev/null; then
  "$KPY" "$TOOLS/route_finish.py" "$CFG" dump route_geom.json 2>&1 | grep -v -E "assert|Debug"
  "$NPY" "$TOOLS/route_finish.py" route route_geom.json route_new.json
  "$KPY" "$TOOLS/route_finish.py" "$CFG" apply route_new.json 2>&1 | grep -v -E "assert|Debug"
  rm -f route_geom.json route_new.json
fi
"$KPY" "$TOOLS/build_pcb.py" "$CFG" sync_fields 2>&1 | grep -v -E "assert|Debug"
"$KCLI" pcb drc --schematic-parity --format json -o drc_report.json "$NAME.kicad_pcb" > /dev/null || true
python3 -c "import json;d=json.load(open('drc_report.json'));v=d.get('violations',[]);print('DRC:',sum(x['severity']=='error' for x in v),'errors,',sum(x['severity']=='warning' for x in v),'warnings,',len(d.get('unconnected_items',[])),'unconnected,',len(d.get('schematic_parity',[])),'parity')"
mkdir -p fab
"$KCLI" pcb export gerbers -o fab/gerbers/ "$NAME.kicad_pcb" > /dev/null
"$KCLI" pcb export drill -o fab/gerbers/ "$NAME.kicad_pcb" > /dev/null
(cd fab && rm -f "${NAME}_gerbers.zip" && zip -qr "${NAME}_gerbers.zip" gerbers)
"$KCLI" pcb export pos --format csv --units mm -o "fab/${NAME}_pos.csv" "$NAME.kicad_pcb" > /dev/null
"$KCLI" sch export bom -o "fab/${NAME}_bom.csv" "$NAME.kicad_sch" > /dev/null 2>&1 || true
"$KCLI" pcb export step -o "$NAME.step" "$NAME.kicad_pcb" > /dev/null 2>&1 || true
echo FLOW_DONE
