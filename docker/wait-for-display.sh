#!/usr/bin/env bash
set -euo pipefail
for attempt in {1..100}; do
  if xdpyinfo -display "${DISPLAY:?DISPLAY is required}" >/dev/null 2>&1; then
    exec "$@"
  fi
  sleep 0.2
done
echo "X display $DISPLAY did not become ready within 20 seconds." >&2
exit 1
