#!/usr/bin/env bash
# Builds the deployable directory: exactly the nine assets the page serves, and
# nothing else. Run `bash web/build.sh` first, then this, then deploy/dist is
# what a static host (or deploy/manifest.json) should receive.
set -euo pipefail
cd "$(dirname "$0")/.."

assets=(index.html index.js index.wasm showcase.css core.js scenes.js render.js instrument.js showcase.js)
rm -rf deploy/dist
mkdir -p deploy/dist
for asset in "${assets[@]}"; do
  [ -f "build/web/$asset" ] || { echo "missing build/web/$asset - run bash web/build.sh first" >&2; exit 1; }
  cp "build/web/$asset" deploy/dist/
done

count=$(ls -1 deploy/dist | wc -l)
[ "$count" -eq 9 ] || { echo "deploy/dist holds $count files, expected 9" >&2; exit 1; }
[ ! -f deploy/dist/server.log ] || { echo "refusing to ship a log" >&2; exit 1; }
echo "deploy/dist: $count assets, $(du -sh deploy/dist | cut -f1)"
