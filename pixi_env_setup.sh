#!/usr/bin/bash
set -e

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ZENOH_CONFIG_OVERRIDE="${ZENOH_CONFIG_OVERRIDE:-transport/shared_memory/enabled=true;transport/shared_memory/transport_optimization/pool_size=536870912}"
