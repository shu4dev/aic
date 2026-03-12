#!/usr/bin/bash
set -e
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE="transport/shared_memory/enabled=false"

if [ -f "$CONDA_PREFIX/bin/pip" ]; then
    "$CONDA_PREFIX/bin/pip" install --upgrade "jax[cuda12_pip]" \
        -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html
fi
