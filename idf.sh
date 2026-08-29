#!/usr/bin/env bash
# Run idf.py (or any command) inside the ESP-IDF container.
#
#   ./idf.sh build                 # build
#   ./idf.sh -p $ESPPORT flash     # flash
#   ./idf.sh monitor               # serial monitor (Ctrl-] to quit)
#   ./idf.sh shell                 # interactive shell with IDF exported
#
# Nothing is installed on the host: toolchain, python env and esptool all live
# in the image. The project directory is bind-mounted, build artifacts are
# written back as the calling user.
set -euo pipefail

IDF_IMAGE="${IDF_IMAGE:-espressif/idf:v6.1}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${ESPPORT:-/dev/ttyACM0}"

args=(--rm -i
      -v "$PROJECT_DIR:/project" -w /project
      -u "$(id -u):$(id -g)" -e HOME=/tmp
      -e ESPPORT="$PORT" -e ESPBAUD="${ESPBAUD:-460800}")

[ -t 0 ] && args+=(-t)

# Pass the serial adapter through. The device node is root:uucp on the host, so
# join its group inside the container instead of running as root.
if [ -e "$PORT" ]; then
    dev="$(readlink -f "$PORT")"
    args+=(--device "$dev:$dev" --group-add "$(stat -Lc %g "$dev")")
    [ "$dev" != "$PORT" ] && args+=(--device "$PORT:$PORT")
else
    echo "warning: $PORT not present, running without serial access" >&2
fi

if [ "${1:-}" = "shell" ]; then
    shift
    exec docker run "${args[@]}" "$IDF_IMAGE" bash "$@"
fi

exec docker run "${args[@]}" "$IDF_IMAGE" idf.py "$@"
