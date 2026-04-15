#!/bin/bash
# =============================================================================
# WSL2 Environment Setup for Gazebo / RViz2 GUI Rendering
# Source this script before launching the simulation:
#   source setup_env.sh
# =============================================================================

# export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=0
export MESA_GL_VERSION_OVERRIDE=3.3

echo "[setup_env] DISPLAY=${DISPLAY}"
echo "[setup_env] LIBGL_ALWAYS_INDIRECT=${LIBGL_ALWAYS_INDIRECT}"
echo "[setup_env] MESA_GL_VERSION_OVERRIDE=${MESA_GL_VERSION_OVERRIDE}"
echo "[setup_env] WSL2 GUI environment configured."
