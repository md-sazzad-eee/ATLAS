#!/bin/bash
# Board B: place -> Freerouting -> import + pours -> completion passes -> DRC  (run on macOS)
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE/.."
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
JAVA=${JAVA:-java}   # any Java 21+
FR=${FR:-$HOME/tools/freerouting-2.1.0.jar}
route() { rm -f "$2"; "$JAVA" -Xmx6g -Djava.awt.headless=true -jar "$FR" -de "$1" -do "$2" -mp 60 \
            --gui.enabled=false --api_server.enabled=false > freerouting.log 2>&1; }
"$KPY" design/build_board_b_pcb.py place 2>&1 | grep -v assert
route atlas_agent_ctrl.dsn atlas_agent_ctrl.ses
"$KPY" design/build_board_b_pcb.py import_ses atlas_agent_ctrl.ses 2>&1 | grep -v assert
"$KPY" design/build_board_b_pcb.py bridge 2>&1 | grep -v assert
for i in 1 2 3; do
  U=$("$KPY" design/build_board_b_pcb.py unconnected 2>&1 | grep UNCONNECTED | awk '{print $2}')
  echo "unconnected after pass: $U"
  [ "$U" = "0" ] && break
  "$KPY" design/build_board_b_pcb.py export_routed atlas_agent_ctrl_complete.dsn 2>&1 | grep -v assert
  route atlas_agent_ctrl_complete.dsn atlas_agent_ctrl_complete.ses
  "$KPY" design/build_board_b_pcb.py import_ses atlas_agent_ctrl_complete.ses 2>&1 | grep -v assert
  "$KPY" design/build_board_b_pcb.py bridge 2>&1 | grep -v assert
done
/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli pcb drc --schematic-parity --format json -o drc_report.json atlas_agent_ctrl.kicad_pcb
echo PIPELINE_DONE
