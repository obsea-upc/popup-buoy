#!/bin/bash
# Downloads today's Kineis AOP from the CLS API as the AOP.txt the buoy reads.
#
#   ./tools/get_aop.sh hardware/tests/2026-10-01
#
# retrieve-kineis-aop answers a packed binary by default; ?format=text gives the
# tab-separated table the firmware parses (satellite, hex id, ADCS address, downlink
# and uplink status, bulletin date and time, semi-major axis, inclination, node
# longitude and drift, period, semi-major axis drift). Found in the API's OpenAPI
# spec, 28 Sep 2026. Other formats: json, argos (legacy json, with argosType).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="${1:?usage: $0 <campaign folder>}"
TMP="$(mktemp)"
CLS_ACCEPT=application/octet-stream "$HERE/cls.sh" "retrieve-kineis-aop?format=text" '{}' > "$TMP"

# Refuse anything that is not the 17-column table, so a login error or an API change
# never ends up on a buoy's card.
lines=$(grep -c . "$TMP" || true)
bad=$(awk -F'\t' 'NF && NF != 17' "$TMP" | wc -l)
if [ "$lines" -lt 10 ] || [ "$bad" -ne 0 ]; then
  echo "AOP rejected: $lines lines, $bad not 17 columns" >&2; head -c 300 "$TMP" >&2; rm -f "$TMP"; exit 1
fi
mv "$TMP" "$DEST/AOP.txt"
oldest=$(awk -F'\t' '{printf "%s-%s-%s %s:%s\n", $6,$7,$8,$9,$10}' "$DEST/AOP.txt" | sort | head -1)
echo "AOP.txt: $lines satellites, oldest bulletin $oldest UTC -> $DEST/AOP.txt"
