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

# Run from the repo root so this covers both mp2p_icp_core and mp2p_icp_viz:
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

find \
    mp2p_icp_core/apps \
    mp2p_icp_core/mp2p_icp \
    mp2p_icp_core/mp2p_icp_map \
    mp2p_icp_core/mp2p_icp_filters \
    mp2p_icp_core/mp2p_icp_common \
    mp2p_icp_core/tests \
    mp2p_icp_viz/apps \
    \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \) \
  -print0 | xargs -0 -r -t clang-format-14 "${MODE[@]}"
