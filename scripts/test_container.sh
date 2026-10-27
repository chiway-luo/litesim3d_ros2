#!/usr/bin/env bash
# Integration smoke test for an already built litesim3d:humble image.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output="${1:-$(mktemp -d "${TMPDIR:-/tmp}/litesim-container.XXXXXX")}"
mkdir -p "$output"
export LITESIM3D_WEB_PORT="${LITESIM3D_WEB_PORT:-16080}"
compose=(docker compose -p "litesim-smoke-$$" -f "$project_dir/compose.yaml" --profile headless)

cleanup() {
  result=$?
  trap - EXIT
  "${compose[@]}" logs --no-color > "$output/containers.log" 2>&1 || true
  "${compose[@]}" down --timeout 45 || result=1
  printf 'Container smoke evidence: %s\n' "$output"
  exit "$result"
}
trap cleanup EXIT

for service in headless desktop; do
  "${compose[@]}" up -d --no-build "$service"
  ready=0
  for attempt in {1..20}; do
    if "${compose[@]}" exec -T "$service" litesim-env litesim-check \
        --duration 3 --require-imu > "$output/$service-ready.json" 2> "$output/$service-ready.log"; then
      ready=1
      break
    fi
    sleep 2
  done
  if [[ "$ready" != 1 ]]; then
    cat "$output/$service-ready.log" >&2
    echo "$service did not publish the required simulation data." >&2
    exit 1
  fi
  "${compose[@]}" exec -T "$service" litesim-env litesim-check \
    --drive --linear 0.1 --angular 0.0 --duration 8 --require-imu --require-motion \
    > "$output/$service-motion.json"
  if [[ "$service" == desktop ]]; then
    "${compose[@]}" exec -T desktop python3 -c \
      'from pathlib import Path; files = list(Path("/tmp").glob("litesim-sdf-*/viewer.ready")); assert len(files) == 1 and files[0].read_text().strip(), files; print(files[0].read_text())' \
      > "$output/desktop-render-ready.txt"
    "${compose[@]}" exec -T desktop supervisorctl -c /etc/litesim3d/supervisord.conf status \
      > "$output/desktop-processes.txt"
    "${compose[@]}" exec -T desktop glxinfo -B > "$output/desktop-opengl.txt"
    "${compose[@]}" exec -T desktop python3 -c \
      'import socket; s = socket.create_connection(("127.0.0.1", 5900), timeout=5); banner = s.makefile("rb").read(12); assert banner.startswith(b"RFB "), banner; print(banner.decode().strip())' \
      > "$output/desktop-vnc.txt"
    curl --fail --silent --show-error --connect-timeout 5 --max-time 10 \
      "http://127.0.0.1:$LITESIM3D_WEB_PORT/vnc.html" \
      > "$output/desktop-vnc.html"
  fi
  container_id="$("${compose[@]}" ps -q "$service")"
  "${compose[@]}" stop --timeout 45 "$service"
  exit_code="$(docker inspect --format '{{.State.ExitCode}}' "$container_id")"
  printf '%s\n' "$exit_code" > "$output/$service-exit-code.txt"
  if [[ "$exit_code" != 0 && "$exit_code" != 130 ]]; then
    echo "$service failed to stop cleanly (exit $exit_code)." >&2
    exit 1
  fi
done
