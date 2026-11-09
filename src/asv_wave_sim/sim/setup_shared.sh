#!/bin/bash
# Shared simulator install for a machine with several accounts (the sim Jetson). Moves this
# account's ArduPilot, ArduPilot venv, ArduPilot Gazebo plugin, TensorRT engine cache and
# message_node protobuf into one copy under /opt/aqua, and keeps one shared copy of the repo's git
# history there that everyone's clone borrows from. config.env picks /opt/aqua up automatically.
#
#   setup_shared.sh               run once, as the account whose setup_sim.sh install works; asks for sudo
#   setup_shared.sh --link PATH   also make another clone borrow the shared history (any user's
#                                 clone; uncommitted work and local branches are kept). Repeatable.
#   setup_shared.sh --check       report what is shared and what is still per-user; change nothing
#
# Safe to re-run: every step checks first. Never delete /opt/aqua/autonomy-workspace-26-27.git:
# linked clones read their history from it.
set -eu
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$SIM_DIR/../../.." && pwd)"
SHARED="${AQUA_SHARED:-/opt/aqua}"
GROUP=aqua
MIRROR="$SHARED/autonomy-workspace-26-27.git"

CHECK=0
LINKS=("$WS")
while [ $# -gt 0 ]; do
  case "$1" in
    --check) CHECK=1 ;;
    --link) shift; LINKS+=("$(cd "$1" && pwd)") ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done
[ "$(id -u)" -ne 0 ] || { echo "Run as your own account, not root (it uses sudo where needed)." >&2; exit 1; }

step() { echo; echo "=== $1"; }
size() { du -sh "$1" 2>/dev/null | cut -f1; }

# Group-writable for the aqua group, readable by everyone, new files inherit the group.
share_perms() {
  sudo chgrp -R "$GROUP" "$1"
  sudo chmod -R g+rwX,o+rX "$1"
  sudo find "$1" -type d -exec chmod g+s {} +
}

# Moves $HOME/<src> to $SHARED/<dst> (same filesystem, so a rename).
MOVED=0
move_in() {
  local src="$1" dst="$2"
  if [ -e "$dst" ]; then echo "  [ok]      $dst ($(size "$dst"))"; [ -e "$src" ] && echo "  [note]    $src still exists; remove it once the shared copy works"; return 0; fi
  if [ ! -e "$src" ]; then echo "  [missing] $src (not installed for $(id -un))"; return 0; fi
  [ $CHECK = 1 ] && { echo "  [todo]    $src -> $dst ($(size "$src"))"; return 0; }
  sudo mkdir -p "$(dirname "$dst")"
  sudo mv "$src" "$dst"
  MOVED=1
  echo "  [done]    $src -> $dst"
}

# Venvs (activate, script shebangs) and the waf/CMake caches store absolute paths; point every text
# file that names the old locations at the shared ones. Binaries are left alone (only a rebuild
# reads their stored paths).
fix_paths() {
  local d dirs=()
  for d in ardupilot venv-ardupilot ardupilot_gazebo; do [ -d "$SHARED/$d" ] && dirs+=("$SHARED/$d"); done
  [ ${#dirs[@]} -gt 0 ] || return 0
  sudo grep -rlIZ --exclude-dir=.git -E "$HOME/(venv-ardupilot|ardupilot)" "${dirs[@]}" 2>/dev/null |
    xargs -0 -r sudo sed -i -E "s#$HOME/(venv-ardupilot|ardupilot)#$SHARED/\\1#g"
  for d in "${dirs[@]}" "$SHARED/cache"; do [ -d "$d" ] && share_perms "$d"; done
  echo "  [done]    paths inside the moved installs now point at $SHARED"
}

step "Group $GROUP and $SHARED"
users=$(getent passwd | awk -F: '$3>=1000 && $3<60000 && $6 ~ /^\/home\// {print $1}')
if [ $CHECK = 1 ]; then
  getent group "$GROUP" >/dev/null && echo "  [ok]      group $GROUP: $(getent group "$GROUP" | cut -d: -f4)" || echo "  [todo]    group $GROUP"
else
  getent group "$GROUP" >/dev/null || sudo groupadd "$GROUP"
  for u in $users; do id -nG "$u" | tr ' ' '\n' | grep -qx "$GROUP" || sudo usermod -aG "$GROUP" "$u"; done
  sudo install -d -m 2775 -g "$GROUP" "$SHARED" "$SHARED/cache"
  echo "  [done]    group $GROUP (every /home account), $SHARED"
fi

step "Shared tools"
move_in "$HOME/ardupilot" "$SHARED/ardupilot"
move_in "$HOME/venv-ardupilot" "$SHARED/venv-ardupilot"
move_in "$HOME/ardupilot_gazebo" "$SHARED/ardupilot_gazebo"
move_in "$HOME/.cache/aquatonomous/engines" "$SHARED/cache/engines"
move_in "$HOME/.cache/aquatonomous/message_node_pydeps" "$SHARED/cache/message_node_pydeps"
[ $MOVED = 1 ] && fix_paths
if [ $CHECK = 0 ] && [ -x "$SHARED/venv-ardupilot/bin/python" ]; then
  "$SHARED/venv-ardupilot/bin/python" -c 'import pymavlink, MAVProxy' 2>/dev/null &&
    echo "  [ok]      venv imports pymavlink + MAVProxy" ||
    echo "  [FAILED]  $SHARED/venv-ardupilot cannot import pymavlink/MAVProxy"
fi

# ArduPilot's installer adds lines for ~/venv-ardupilot and ~/ardupilot to ~/.bashrc.
if grep -qE "($HOME|\\\$HOME|~)/(venv-ardupilot|ardupilot)" "$HOME/.bashrc" 2>/dev/null; then
  if [ $CHECK = 1 ]; then echo "  [todo]    ~/.bashrc still points at ~/ardupilot / ~/venv-ardupilot"
  else
    cp "$HOME/.bashrc" "$HOME/.bashrc.before-aqua-shared"
    sed -i -E "s#($HOME|\\\$HOME|~)/(venv-ardupilot|ardupilot)#$SHARED/\\2#g" "$HOME/.bashrc"
    echo "  [done]    ~/.bashrc now uses $SHARED (backup: ~/.bashrc.before-aqua-shared)"
  fi
fi

step "Shared git history ($MIRROR)"
if [ -d "$MIRROR" ]; then echo "  [ok]      $MIRROR ($(size "$MIRROR"))"
elif [ $CHECK = 1 ]; then echo "  [todo]    $MIRROR from $WS"
else
  # Cloned from this checkout (no network or GitHub login needed), then moved into place.
  tmp="$HOME/.aqua-shared-history.git"
  rm -rf "$tmp"
  git clone -q --mirror "$WS" "$tmp"
  git -C "$tmp" remote set-url origin "$(git -C "$WS" remote get-url origin)"
  # Linked clones read objects from here, so nothing may ever be pruned from it.
  git -C "$tmp" config core.sharedRepository group
  git -C "$tmp" config gc.pruneExpire never
  git -C "$tmp" config gc.reflogExpireUnreachable never
  sudo mv "$tmp" "$MIRROR"
  share_perms "$MIRROR"
  git config --system --get-all safe.directory 2>/dev/null | grep -qxF "$MIRROR" ||
    sudo git config --system --add safe.directory "$MIRROR"
  echo "  [done]    $MIRROR ($(size "$MIRROR"))"
fi

step "Clones borrowing the shared history"
for repo in "${LINKS[@]}"; do
  [ -d "$repo/.git/objects" ] || { echo "  [skip]    $repo is not a clone with its own .git"; continue; }
  alt="$repo/.git/objects/info/alternates"
  if grep -qxF "$MIRROR/objects" "$alt" 2>/dev/null; then echo "  [ok]      $repo (.git $(size "$repo/.git"))"; continue; fi
  if [ $CHECK = 1 ]; then echo "  [todo]    $repo (.git $(size "$repo/.git"))"; continue; fi
  owner="$(stat -c %U "$repo")"
  as=(); [ "$owner" = "$(id -un)" ] || as=(sudo -u "$owner")
  before="$(size "$repo/.git")"
  "${as[@]}" sh -c 'mkdir -p "$(dirname "$1")" && echo "$2" >> "$1"' _ "$alt" "$MIRROR/objects"
  # Repack keeping only the objects the shared history does not have (local commits, stashes).
  "${as[@]}" git -C "$repo" repack -a -d -l -q
  "${as[@]}" git -C "$repo" prune-packed
  if "${as[@]}" git -C "$repo" fsck --connectivity-only --no-dangling --no-progress >/dev/null 2>&1; then
    echo "  [done]    $repo (.git $before -> $(size "$repo/.git"))"
  else
    echo "  [FAILED]  $repo: git fsck reports missing objects; do not delete anything, ask for help"
  fi
done

echo
[ $CHECK = 1 ] && echo "Check only; nothing changed." || cat <<EOF
Done. Group membership applies from the next login (log out and back in, or run: newgrp $GROUP).
New clones: git clone --reference $MIRROR <repo url> ~/autonomy-workspace-26-27
Then in that clone: src/asv_wave_sim/sim/setup_sim.sh   (builds only the workspace; ArduPilot is shared)
EOF
