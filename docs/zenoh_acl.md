## Updating zenoh router ACL

Prepare output directory

```bash
mkdir <workspace-root>/sros # or any directory you want
```

Run the evaluator

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py launch_rviz:=false gazebo_gui:=false
```

Generate SROS2 policy file

```bash
ros2 security generate_policy aic_policy.xml
```

Generate base zenoh config

```bash
# apt install ros-kilted-zenoh-security-tools
ros2 run zenoh_security_tools generate_configs -p aic_policy.xml -r ../src/aic/docker/aic_eval/zenoh_router_config.json5 -c ../src/aic/docker/aic_eval/zenoh_router_config.json5 -d 0
```

This will generate a bunch of zenoh config files, one for each node in the SROS2 policy. What you need to do now is to use the generated config files as a reference to update the [config](../docker/aic_eval/zenoh_router_config.json5) used in `aic_eval` image.

The generated config contains settings to allow outgoing publications and incoming subscriptions. But because we are puting the acl on the router with peer brokering. We actually need incoming publications and outgoing subscriptions as well.

Add the topics that you want to allow to one or more of the following

* allow_all
* outgoing_publications
* incoming_subscriptions
* incoming_publications
* outgoing_subscriptions

If you want a topic to _only_ be able to be published by the evaluator to the participant, put it in `outgoing_publications`, `incoming_subscriptions`, `incoming_publications` and `outgoing_subscriptions`.

If you want a topic to be able to be published by anyone, put it in `allow_all`. This includes services and actions which are bidirectional.

How it works is that the router essentially acts as a proxy for the evaluator. When you want to allow the evaluator to publish a topic, the router will *subscribe* to the topic, and *publish* it to the participant. 

So it will:

1. Receive an incoming subscription from the participant.
2. Send an outgoing subscription to the evaluator.
3. Receive an incoming publication from the evaluator.
4. Send an outgoing publication to the participant.

## Quick Testing

> [!warning]
> This is not replace a full test with `docker compose up`.

Start eval router

```bash
. src/aic/docker/acl/zenoh_config_eval_router.sh
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Start evaluator

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py launch_rviz:=false gazebo_gui:=false ground_truth:=false start_aic_engine:=true shutdown_on_aic_engine_exit:=true
```

Start model router

```bash
. src/aic/docker/acl/zenoh_config_model_router.sh
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Start example model

```bash
. src/aic/docker/acl/zenoh_config_model.sh
ros2 run aic_model aic_model --ros-args -p policy:=aic_example_policies.ros.WaveArm
```
