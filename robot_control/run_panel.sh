#!/bin/bash
# Start (or restart) the leg control panel in the background.
#   ./run_panel.sh --sim     simulated motors
#   ./run_panel.sh           real motors
#   ./run_panel.sh stop
# Log: /tmp/leg_server.log    Panel: http://192.168.55.1:8080

cd "$(dirname "$0")"
pkill -f "python3 leg_server.py" && sleep 1
[ "$1" = "stop" ] && echo "panel stopped" && exit 0
setsid nohup python3 leg_server.py "$@" > /tmp/leg_server.log 2>&1 < /dev/null &
sleep 2
cat /tmp/leg_server.log
