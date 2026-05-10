#!/usr/bin/env bash
# Usage: formatter.sh [--check]
#   (default) Reformat all C/C++ sources in-place with clang-format-14.
#   --check   Dry-run: exit non-zero if any file would be reformatted.

set -euo pipefail

if [ "${1:-}" = "--check" ]; then
  MODE=(--dry-run --Werror)
else
  MODE=(-i)
fi

find \
    apps \
    mp2p_icp \
    mp2p_icp_map \
    mp2p_icp_filters \
    mp2p_icp_common \
    tests \
    \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \) \
  -print0 | xargs -0 -r -t clang-format-14 "${MODE[@]}"
