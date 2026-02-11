#!/usr/bin/bash

HERE="$(dirname "${BASH_SOURCE[0]}")"
export ZENOH_ROUTER_CONFIG_URI="$(readlink -f "$HERE/../aic_eval/zenoh_router_config.json5")"
CREDENTIALS="$(readlink -f "$HERE")/credentials.txt"
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/[::]:7448"]'
ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/user="eval-router"'
ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/password="CHANGE_IN_PROD"'
ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/dictionary_file="'"$CREDENTIALS"'"'
export ZENOH_CONFIG_OVERRIDE
