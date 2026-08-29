#!/usr/bin/env bash
# Run scripts/console.py inside the ESP-IDF container, which has pyserial.
#
#   ./wf.sh info
#   ./wf.sh --reset 'scan 10' devs
#   ./wf.sh 'connect name DL' 'probe 30@60' 'rec dump@120'
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$DIR/idf.sh" shell -lc "python3 scripts/console.py $(printf '%q ' "$@")"
