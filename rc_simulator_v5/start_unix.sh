#!/bin/sh
cd "$(dirname "$0")" || exit 1
exec python3 rc_simulator_v5.py "$@"
