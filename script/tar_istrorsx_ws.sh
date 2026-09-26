#!/bin/bash
# Backs up ~/istrorsx_ws into a dated, auto-lettered .tgz in the home
# directory (istrorsx_ws_YYMMDDx.tgz, x = first free letter for today),
# excluding everything make_clean.sh would otherwise delete first (build/
# install/log -- colcon/CMake build output; logout/out/ramdisk -- runtime
# log/image/telemetry dirs, see make_clean.sh's own comment) plus archive/
# (run dumps moved aside by script/mvout.sh -- tens of MB each, and unlike
# the dirs above make_clean.sh does NOT delete them, so they accumulate)
# and any venv directories (.venv*, e.g. script/visionn_segfb0_server/.venv_rosvm) and
# __pycache__. Same "keep the backup small" goal as make_clean.sh, but
# non-destructively -- nothing is actually deleted, tar just skips these
# directories.
#
# Still worth running even though the project is on GitHub now
# (github.com/lnx-git/istrorsx, see doc/ai/07_github_setup.md): the two
# backups are complementary, not redundant. This archive carries the things
# .gitignore deliberately keeps OUT of the repo -- doc/istrobtx_2025/ (the
# legacy source being ported, ~61 MB) and doc/ai/claude/ (archived session
# exports, ~31 MB).
#
# .git/ is deliberately NOT archived (~48 MB, about a third of the tarball):
# the history lives on GitHub, and repeating it in every dated .tgz just
# inflates them. The trade-off: an extracted tarball is a plain directory,
# not a git repo -- to resume work, clone from GitHub and copy the two
# non-git directories above across. And since the history is only safe if it
# actually reached GitHub, the check below warns (without stopping) when
# there are uncommitted or unpushed changes at backup time.
#
# Also excludes CMakeFiles/CMakeCache.txt/cmake_install.cmake wherever they
# appear (e.g. a locally-built legacy test client under
# doc/istrobtx_2025/script/visionn_client/) -- deliberately bare
# (unanchored) patterns, since their names are CMake-internal and
# unambiguous. Not "build"/"install"/"log"/"lib" as bare patterns though --
# this workspace has legitimate non-build directories with those exact
# names (e.g. script/install/, which holds real install scripts, not build
# output), so those stay full-path-anchored above instead.
#
# (A stray nested src/istrorsx_hw/build|install|log -- a one-off colcon
# build accidentally run from inside that package directory instead of the
# workspace root, back on 2026-08-23 -- used to need its own excludes here
# too; deleted 2026-08-27 instead, since it was pure regenerable build
# output with no reason to keep it around at all.)
#
# Uses ~ rather than a hardcoded /home/ubuntu path on purpose -- this
# workspace's own convention (see doc/ai/04_installation.md) is that the
# final Jetson uses /home/istrobotics instead, so scripts must not assume
# the exact home directory path.
set -e

cd ~

DATE=$(date +%y%m%d)

SUFFIX=""
for c in {a..z}; do
    if [ ! -f "istrorsx_ws_${DATE}${c}.tgz" ]; then
        SUFFIX="$c"
        break
    fi
done

if [ -z "$SUFFIX" ]; then
    echo "Error: all letters a-z are already used for istrorsx_ws_${DATE}*.tgz" >&2
    exit 1
fi

OUTFILE="istrorsx_ws_${DATE}${SUFFIX}.tgz"

# Warn -- but never block -- if work would not be recoverable from GitHub.
# Since .git/ is excluded (see header), anything not pushed exists ONLY in
# the working tree, and a restore from this tarball would lose its history.
# A backup taken precisely because the tree is messy is a legitimate thing
# to want, so this only informs.
if [ -d istrorsx_ws/.git ]; then
    DIRTY=$(git -C istrorsx_ws status --porcelain 2>/dev/null | wc -l || echo 0)
    UNPUSHED=$(git -C istrorsx_ws log --branches --not --remotes --oneline 2>/dev/null | wc -l || echo 0)
    if [ "$DIRTY" -gt 0 ] || [ "$UNPUSHED" -gt 0 ]; then
        echo "WARNING: .git/ is not archived, and this workspace has" >&2
        [ "$DIRTY"    -gt 0 ] && echo "         - $DIRTY uncommitted change(s)" >&2
        [ "$UNPUSHED" -gt 0 ] && echo "         - $UNPUSHED commit(s) not pushed to any remote" >&2
        echo "         Those are NOT recoverable from GitHub. Consider" >&2
        echo "         committing and pushing first. Continuing anyway..." >&2
        echo >&2
    fi
fi

echo "Creating ~/$OUTFILE ..."
tar --exclude='.venv*' \
    --exclude='istrorsx_ws/build' \
    --exclude='istrorsx_ws/install' \
    --exclude='istrorsx_ws/log' \
    --exclude='istrorsx_ws/logout' \
    --exclude='istrorsx_ws/out' \
    --exclude='istrorsx_ws/ramdisk' \
    --exclude='istrorsx_ws/archive' \
    --exclude='istrorsx_ws/.git' \
    --exclude='__pycache__' \
    --exclude='CMakeFiles' \
    --exclude='CMakeCache.txt' \
    --exclude='cmake_install.cmake' \
    -zvcf "$OUTFILE" istrorsx_ws/

echo "Done."
ls -lh "$OUTFILE"
