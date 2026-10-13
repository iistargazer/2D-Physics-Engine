#!/usr/bin/env bash
# Builds the browser prototype. Requires the Emscripten SDK on PATH:
#
#   git clone https://github.com/emscripten-core/emsdk.git build/emsdk
#   cd build/emsdk && python emsdk.py install latest && python emsdk.py activate latest
#   source build/emsdk/emsdk_env.sh
#
# Produces build/web/index.html + index.js + index.wasm. The physics is 100% the
# same C++ headers as the native tests; only the renderer is JavaScript.
set -euo pipefail
cd "$(dirname "$0")/.."
command -v em++ >/dev/null || { echo 'em++ not found. Install and activate the Emscripten SDK first (see the comment above).' >&2; exit 1; }
mkdir -p build/web
em++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -I. web/main.cpp \
  --shell-file web/shell.html \
  -sALLOW_MEMORY_GROWTH=1 \
  -sEXPORTED_FUNCTIONS='["_main","_tick_js","_select_scene","_spawn_box","_spawn_circle","_toggle_pause","_step_once","_set_slow","_toggle_debug","_is_paused","_debug_enabled","_begin_drag","_update_drag","_end_drag","_frame_buffer","_frame_size","_stats_buffer","_stats_size","_profile_enabled","_set_profile","_iterations","_set_iterations","_position_correction","_set_position_correction","_sleeping_enabled","_set_sleeping","_gravity_y","_set_gravity","_solver_mode","_sim_steps"]' \
  -sEXPORTED_RUNTIME_METHODS='["HEAPF32"]' \
  -o build/web/index.html
cp web/showcase.css web/core.js web/scenes.js web/render.js web/instrument.js web/showcase.js build/web/
echo "Built build/web/index.html (with showcase.css + core.js, scenes.js, render.js, instrument.js, showcase.js)"
