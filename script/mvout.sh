#!/bin/bash
# Archives one run's output (out/ + logout/) into archive/<dir_name>/ and
# truncates the live logs, ready for the next run.
#
#   script/mvout.sh 260911_park
#
# Dumps land under archive/ rather than the workspace root so that one
# careless "git add -A" can't pull tens of MB of .jpg and .log into the repo
# -- archive/ is ignored wholesale in .gitignore. The per-dump .gitignore
# written below is the second line of defence, for the case where this ever
# runs against a tree whose .gitignore doesn't have that rule (a fresh clone
# on another machine, say): it makes the dump self-ignoring wherever it sits.
# It is written BEFORE anything is moved in, so there is no window in which
# the directory exists full of files and unprotected.
#
# No "set -e" on purpose: the mv/cp lines below are allowed to fail when a run
# produced no images or no visionn log, and the truncation at the end must
# still happen -- that was the original script's behaviour and it is correct.

if [ -z "$1" ]; then
  echo "usage:  mvout dir_name"
  exit 1
fi

cd ~/istrorsx_ws

if [ ! -d "out" ]; then
  echo "error: out directory doesn't exist!"
  exit 1
fi

if [ ! -d "logout" ]; then
  echo "error: logout directory doesn't exist!"
  exit 1
fi

TARGET="archive/$1"

if [ -d "$TARGET" ]; then
  echo "error: target directory \"$TARGET\" already exists!"
  exit 1
fi

mkdir -p "$TARGET"

if [ ! -d "$TARGET" ]; then
  echo "error: unable to create target directory!"
  exit 1
fi

# Ignore everything in here except this file itself -- same pattern as
# archive/.gitignore one level up.
cat > "$TARGET/.gitignore" <<'EOF'
*
!.gitignore
EOF

# find -exec + and not a glob: out/ holds tens of thousands of images after a
# long drive and "mv out/*" dies with "Argument list too long" (hit in the
# field 2026-09-12, and it moved nothing while still reporting success).
find out    -maxdepth 1 -type f ! -name '.gitkeep' -exec mv -t "$TARGET" {} +
find logout -maxdepth 1 -type f -name 'istro*'     -exec mv -t "$TARGET" {} +
cp logout/visionn_server* "$TARGET"
cp logout/logp.log "$TARGET"
cp logout/gpspipe.log "$TARGET"

# Say so if anything stayed behind, instead of printing "done" over a failure.
left=$(find out -maxdepth 1 -type f ! -name '.gitkeep' | wc -l)
if [ "$left" -gt 0 ]; then
  echo "warning: $left file(s) still in out/ -- archive is incomplete!"
fi

#visionn_server/visionn_server_logtrunc.sh
truncate -s 0 logout/visionn_server.log
date -Iseconds >> logout/visionn_server.log
truncate -s 0 logout/visionn_server2.log
date -Iseconds >> logout/visionn_server2.log

#script/logp_logtrunc.sh
truncate -s 0 logout/logp.log
date -Iseconds >> logout/logp.log

# raw NMEA only, no date line: gpspipe (_gpsd_run.sh) keeps appending to it
truncate -s 0 logout/gpspipe.log

echo "done: files were moved to \"$TARGET\"..."
