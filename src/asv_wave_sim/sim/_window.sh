#!/bin/bash
# One run_competition.sh window: runs the command, then waits so its output stays readable.
name="$1"; shift
cd "$(dirname "${BASH_SOURCE[0]}")" || exit 1
bash -c "$*"
echo
echo "[$name exited] press Enter to close"
read -r _
