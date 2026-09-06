#!/usr/bin/env bash
set -euo pipefail

thirdparty_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$thirdparty_dir/.." && pwd)"
build_dir="$workspace_dir/build/thirdparty"
install_dir="$workspace_dir/install/thirdparty"

cmake -S "$thirdparty_dir" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$install_dir"
cmake --build "$build_dir" --parallel
cmake --install "$build_dir"

echo "Bundled solver installed to: $install_dir"
