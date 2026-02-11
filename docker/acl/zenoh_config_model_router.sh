#!/usr/bin/bash

HERE="$(dirname "${BASH_SOURCE[0]}")"
unset ZENOH_ROUTER_CONFIG_URI
CREDENTIALS="$(readlink -f "$HERE")/credentials.txt"
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7448"]'
ZENOH_CONFIG_OVERRIDE+=';routing/router/peers_failover_brokering=true'
ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/user="model-router"'
ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/password="CHANGE_IN_PROD"'
ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/dictionary_file="'"$CREDENTIALS"'"'
export ZENOH_CONFIG_OVERRIDE
