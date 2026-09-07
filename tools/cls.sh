#!/bin/bash
# Calls the CLS telemetry API with a freshly minted token.
#
#   ./tools/cls.sh <endpoint> '<json request>'
#
# e.g.
#   ./tools/cls.sh retrieve-bulk '{"pagination":{"first":500},"retrieveMetadata":true,
#    "retrieveRawData":true,"deviceRefs":["217459"],
#    "fromDatetime":"2026-09-04T05:00:00.001Z","toDatetime":"2026-09-05T05:00:00.001Z",
#    "datetimeFormat":"DATETIME"}'
#
# Credentials come from hardware/credentials.txt (user=... / pass=...), which is
# covered by .gitignore. They are read into variables and never echoed, and the
# token lives about five minutes so it is minted per call rather than cached.
#
# Rebuilt from the version written in August, which lived in a scratch directory
# and did not survive to be used a second time - hence tools/.

set -euo pipefail

AUTH_URL="https://account.groupcls.com/auth/realms/cls/protocol/openid-connect/token"
API_URL="https://api.groupcls.com/telemetry/api/v1"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CRED="$HERE/../hardware/credentials.txt"

if [ $# -lt 2 ]; then
  echo "usage: $0 <endpoint> '<json>'" >&2
  echo "endpoints: retrieve-bulk, retrieve-realtime, retrieve-aop, retrieve-device-list" >&2
  exit 2
fi

ENDPOINT="$1"
REQUEST="$2"

if [ ! -f "$CRED" ]; then
  echo "no credentials at $CRED (expected user=... and pass=... lines)" >&2
  exit 1
fi

# The values are written quoted in the file, and the CRLF line endings leave a
# stray carriage return. Both have to come off or they are sent as part of the
# credential and the server answers "Invalid user credentials", which reads like
# a wrong password rather than a parsing bug.
read_cred() {
  grep -E "^$1=" "$CRED" | head -1 | cut -d= -f2- | tr -d '\r' \
    | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' -e 's/^"\(.*\)"$/\1/' -e "s/^'\(.*\)'$/\1/"
}

USERNAME="$(read_cred user)"
PASSWD="$(read_cred pass)"

if [ -z "$USERNAME" ] || [ -z "$PASSWD" ]; then
  echo "credentials file is missing a user= or pass= line" >&2
  exit 1
fi

# sed rather than jq: jq is not installed on this machine and the token is the
# only field needed, so the script stays dependency-free. Output below is raw
# JSON for the caller to parse.
TOKEN="$(curl -s -X POST "$AUTH_URL" \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -d "grant_type=password" \
  -d "client_id=api-telemetry" \
  --data-urlencode "username=${USERNAME}" \
  --data-urlencode "password=${PASSWD}" \
  | sed -n 's/.*"access_token"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')"

if [ -z "$TOKEN" ]; then
  # Deliberately says nothing about which half was wrong, and never prints them.
  echo "authentication failed - check hardware/credentials.txt" >&2
  exit 1
fi

curl -s -X POST "$API_URL/$ENDPOINT" \
  -H 'accept: application/json' \
  -H "Authorization: Bearer $TOKEN" \
  -H 'Content-Type: application/json' \
  -d "$REQUEST"
