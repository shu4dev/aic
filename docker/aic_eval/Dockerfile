FROM ros:kilted-ros-base

# Add `packages.osrfoundation.org` to the apt sources list:
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install dependencies
RUN mkdir -p /ws_aic/src/aic
COPY --chmod=644 aic.repos /ws_aic/src/aic/
COPY --chmod=u=rwX,go=rX --parents **/package.xml /ws_aic/src/aic
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked --mount=type=cache,target=/var/lib/apt,sharing=locked \
  apt update && apt upgrade -y && \
  # undeclared deps
  apt install python3-numpy && \
  cd /ws_aic/src && \
  vcs import . < /ws_aic/src/aic/aic.repos --recursive && \
  # Install Gazebo dependencies.
  apt -y install $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') && \
  cd /ws_aic && \
  # Install ROS dependencies using rosdep.
  rosdep install --from-paths src --ignore-src --rosdistro kilted -yr --skip-keys "gz-cmake3 DART libogre-dev libogre-next-2.3-dev"

# Build the workspace
COPY --chmod=u=rwX,go=rX --exclude=docker . /ws_aic/src/aic
SHELL ["/bin/bash", "-c"]
RUN --mount=type=cache,target=/ws_aic/build,sharing=locked \
  cd /ws_aic \
  && . /opt/ros/kilted/setup.bash \
  && GZ_BUILD_FROM_SOURCE=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --executor sequential \
  && rm -rf /ws_aic/log /ws_aic/src

COPY --chmod=644 docker/aic_eval/zenoh_router_config.json5 /zenoh_router_config.json5
COPY --chmod=755 <<"EOF" /start_eval.sh
#!/bin/bash
set -e

. /ws_aic/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

should_disable_acl() {
  [[ "$AIC_DISABLE_ACL" == "true" || "$AIC_DISABLE_ACL" == "1" ]]
}

# prepare the credentials file
if ! should_disable_acl; then
  if [[ ! (-n "$AIC_EVAL_PASSWD" && -n "$AIC_EVAL_ROUTER_PASSWD" && -n "AIC_MODEL_PASSWD" && -n "$AIC_MODEL_ROUTER_PASSWD") ]]; then
    echo "AIC_EVAL_PASSWD, AIC_EVAL_ROUTER_PASSWD, AIC_MODEL_PASSWD, AIC_MODEL_ROUTER_PASSWD must be provided"
    exit 1
  fi
  echo "eval:$AIC_EVAL_PASSWD" >> /credentials.txt
  echo "eval-router:$AIC_EVAL_ROUTER_PASSWD" >> /credentials.txt
  echo "model:$AIC_MODEL_PASSWD" >> /credentials.txt
  echo "model-router:$AIC_MODEL_ROUTER_PASSWD" >> /credentials.txt
fi

# start the eval router
unset ZENOH_ROUTER_CONFIG_URI
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/[::]:7448"]'
if ! should_disable_acl; then
  export ZENOH_ROUTER_CONFIG_URI=/zenoh_router_config.json5
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/user="eval-router"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/password="'"$AIC_EVAL_ROUTER_PASSWD"'"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/dictionary_file="/credentials.txt"'
fi
export ZENOH_CONFIG_OVERRIDE
ros2 run rmw_zenoh_cpp rmw_zenohd &
ZENOH_EVAL_ROUTER_PID=$!

# start the model router
unset ZENOH_ROUTER_CONFIG_URI
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7448"]'
ZENOH_CONFIG_OVERRIDE+=';routing/router/peers_failover_brokering=true'
if ! should_disable_acl; then
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/user="model-router"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/password="'"$AIC_MODEL_ROUTER_PASSWD"'"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/dictionary_file="/credentials.txt"'
  export ZENOH_CONFIG_OVERRIDE
else
  unset ZENOH_CONFIG_OVERRIDE
fi
ros2 run rmw_zenoh_cpp rmw_zenohd &
ZENOH_MODEL_ROUTER_PID=$!

trap "kill -SIGINT -$ZENOH_EVAL_ROUTER_PID -$ZENOH_MODEL_ROUTER_PID" EXIT

# start the evaluator
ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/127.0.0.1:7448"]'
if ! should_disable_acl; then
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/user="eval"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/password="'"$AIC_EVAL_PASSWD"'"'
  ZENOH_CONFIG_OVERRIDE+=';transport/auth/usrpwd/dictionary_file="/credentials.txt"'
fi
export ZENOH_CONFIG_OVERRIDE
echo "ZENOH_CONFIG_OVERRIDE=$ZENOH_CONFIG_OVERRIDE"
exec ros2 launch aic_bringup aic_gz_bringup.launch.py "$@"
EOF

COPY --chmod=755 <<"EOF" /entrypoint.sh
#!/bin/bash
set -e

AIC_DISABLE_ACL=${AIC_DISABLE_ACL:-1} exec /start_eval.sh "$@"
EOF

# Contary to the name, `/entrypoint.sh` is not actually the entrypoint.
# The name is kept to maintain compatibility as the old entrypoint script does not have ACL.
ENTRYPOINT ["/start_eval.sh"]
