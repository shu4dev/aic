#!/usr/bin/bash

HERE="$(dirname "${BASH_SOURCE[0]}")"
CREDENTIALS="$(readlink -f "$HERE")/model-credentials.txt"
ZENOH_CONFIG_OVERRIDE='transport/auth/usrpwd/user="model"'
ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/password="CHANGE_IN_PROD"'
ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/dictionary_file="'"$CREDENTIALS"'"'
export ZENOH_CONFIG_OVERRIDE
