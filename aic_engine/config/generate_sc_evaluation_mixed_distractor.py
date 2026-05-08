#!/usr/bin/env python3
"""Generate an SC eval/collection config with a *probabilistic* distractor.

Mixes two trial types in one config based on --distractor-prob:
  * with-distractor:  both sc_rail_0 AND sc_rail_1 populated (one target,
                      one decoy with a different entity_name).
  * no-distractor:    only the target rail populated, other rail empty
                      (matches generate_sc_eval_config.py behaviour).

Lets you run a single eval that probes both "find the SC port when alone"
and "find the named SC port despite a visible decoy" — useful for measuring
the policy's robustness to distractors directly inside one trial set.

Usage:
    python scripts/generate_sc_eval_config_mixed_distractor.py \\
        --trials 30 --seed 888 --distractor-prob 0.5 \\
        --output aic_engine/config/eval_sc_mixed_seed888_60.yaml

    python scripts/generate_sc_eval_config_mixed_distractor.py \\
        --collection --trials 10 --seed 42 --distractor-prob 0.3 \\
        --output aic_engine/config/collect_sc_mixed_10.yaml
"""

import argparse
import random
import textwrap
from pathlib import Path

# ── Randomisation ranges (mirrors the with-distractor variant) ────────────────
BOARD_X   = (0.20, 0.34)
BOARD_Y   = (-0.16, 0.02)
BOARD_Z   = (1.14, 1.14)
BOARD_YAW = (-0.18, 0.18)

SC_RAIL_TRANS = (-0.060, 0.055)

SC_CABLE_OFFSET = {"x": 0.0, "y": 0.015385, "z": 0.04045,
                   "roll": 0.4432, "pitch": -0.4838, "yaw": 1.3303}

SC_ENTITY_NAMES = ["sc_mount_0", "sc_mount_1", "sc_mount_2"]

HOME_JOINTS = {
    "shoulder_pan_joint":  -0.1597,
    "shoulder_lift_joint": -1.3542,
    "elbow_joint":         -1.6648,
    "wrist_1_joint":       -1.6933,
    "wrist_2_joint":        1.5710,
    "wrist_3_joint":        1.4110,
}


def rand(lo, hi):
    return round(random.uniform(lo, hi), 4)


def build_trial(trial_idx: int, rng: random.Random,
                distractor_prob: float) -> tuple[dict, bool]:
    """SC plug → one SC port. distractor_prob picks whether the non-target rail
    is populated. Returns (trial_dict, has_distractor)."""
    target_rail = rng.randint(0, 1)
    has_distractor = rng.random() < distractor_prob

    # Pick 2 distinct entity names so target and distractor never collide
    # in Gazebo (matches sample_config.yaml convention). If no distractor,
    # we still consume the second name so per-trial RNG state is consistent
    # regardless of distractor_prob.
    target_entity, distractor_entity = rng.sample(SC_ENTITY_NAMES, 2)

    board_pose = {
        "x": rand(*BOARD_X),
        "y": rand(*BOARD_Y),
        "z": rand(*BOARD_Z),
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": rand(*BOARD_YAW),
    }

    def sc_rail_block(i: int) -> dict:
        is_target = (i == target_rail)
        if is_target:
            return {
                "entity_present": True,
                "entity_name": target_entity,
                "entity_pose": {
                    "translation": rand(SC_RAIL_TRANS[0], SC_RAIL_TRANS[1]),
                    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                },
            }
        if has_distractor:
            return {
                "entity_present": True,
                "entity_name": distractor_entity,
                "entity_pose": {
                    "translation": rand(SC_RAIL_TRANS[0], SC_RAIL_TRANS[1]),
                    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                },
            }
        return {"entity_present": False}

    trial = {
        "scene": {
            "task_board": {
                "pose": board_pose,
                **{f"nic_rail_{i}": {"entity_present": False} for i in range(5)},
                "sc_rail_0":        sc_rail_block(0),
                "sc_rail_1":        sc_rail_block(1),
                "lc_mount_rail_0":  {"entity_present": False},
                "sfp_mount_rail_0": {"entity_present": False},
                "sc_mount_rail_0":  {"entity_present": False},
                "lc_mount_rail_1":  {"entity_present": False},
                "sfp_mount_rail_1": {"entity_present": False},
                "sc_mount_rail_1":  {"entity_present": False},
            },
            "cables": {
                "cable_1": {
                    "pose": {
                        "gripper_offset": {
                            "x": SC_CABLE_OFFSET["x"],
                            "y": SC_CABLE_OFFSET["y"],
                            "z": SC_CABLE_OFFSET["z"],
                        },
                        "roll":  SC_CABLE_OFFSET["roll"],
                        "pitch": SC_CABLE_OFFSET["pitch"],
                        "yaw":   SC_CABLE_OFFSET["yaw"],
                    },
                    "attach_cable_to_gripper": True,
                    "cable_type": "sfp_sc_cable_reversed",
                },
            },
        },
        "tasks": {
            "task_1": {
                "cable_type": "sfp_sc",
                "cable_name": "cable_1",
                "plug_type":  "sc",
                "plug_name":  "sc_tip",
                "port_type":  "sc",
                "port_name":  "sc_port_base",
                "target_module_name": f"sc_port_{target_rail}",
                "time_limit": 180,
            },
        },
    }
    return trial, has_distractor


def dict_to_yaml(d, indent=0) -> str:
    """Minimal YAML serialiser."""
    lines = []
    pad = "  " * indent
    for k, v in d.items():
        if isinstance(v, dict):
            lines.append(f"{pad}{k}:")
            lines.append(dict_to_yaml(v, indent + 1))
        elif isinstance(v, bool):
            lines.append(f"{pad}{k}: {str(v)}")
        elif isinstance(v, str):
            lines.append(f'{pad}{k}: "{v}"')
        else:
            lines.append(f"{pad}{k}: {v}")
    return "\n".join(lines)


def generate_config(n_trials: int, seed: int, distractor_prob: float,
                    collection: bool = False) -> tuple[str, int]:
    rng = random.Random(seed)
    mode = "collection" if collection else "eval"

    topics = [
        ('"/joint_states"', '"sensor_msgs/msg/JointState"'),
    ]
    if collection:
        topics += [
            ('"/left_camera/image_small"',   '"sensor_msgs/msg/CompressedImage"'),
            ('"/center_camera/image_small"', '"sensor_msgs/msg/CompressedImage"'),
            ('"/right_camera/image_small"',  '"sensor_msgs/msg/CompressedImage"'),
        ]
    topics += [
        ('"/tf"',                              '"tf2_msgs/msg/TFMessage"'),
        ('"/tf_static"',                       '"tf2_msgs/msg/TFMessage"', True),
        ('"/scoring/tf"',                      '"tf2_msgs/msg/TFMessage"'),
        ('"/aic/gazebo/contacts/off_limit"',   '"ros_gz_interfaces/msg/Contacts"'),
        ('"/fts_broadcaster/wrench"',          '"geometry_msgs/msg/WrenchStamped"'),
        ('"/aic_controller/joint_commands"',   '"aic_control_interfaces/msg/JointMotionUpdate"'),
        ('"/aic_controller/pose_commands"',    '"aic_control_interfaces/msg/MotionUpdate"'),
        ('"/scoring/insertion_event"',         '"std_msgs/msg/String"'),
        ('"/aic_controller/controller_state"', '"aic_control_interfaces/msg/ControllerState"'),
    ]
    if collection:
        topics.append(
            ('"/aic_cheatcode/phase"', '"std_msgs/msg/Header"'),
        )

    topic_lines = []
    for t in topics:
        topic_lines.append(f"    - topic:")
        topic_lines.append(f"        name: {t[0]}")
        topic_lines.append(f"        type: {t[1]}")
        if len(t) > 2 and t[2]:
            topic_lines.append(f"        latched: true")
    topics_yaml = "\n".join(topic_lines)

    trial_lines = []
    distractor_count = 0
    for i in range(n_trials):
        trial, has_distractor = build_trial(i, rng, distractor_prob)
        if has_distractor:
            distractor_count += 1
        trial_lines.append(f"  trial_{i + 1}:")
        trial_lines.append(dict_to_yaml(trial, indent=2))
        trial_lines.append("")

    header = f"""# Auto-generated ACT config (SC mixed distractor)
# Seed: {seed}, Trials: {n_trials}, Mode: {mode}
# Distractor prob: {distractor_prob}, Distractor trials: {distractor_count}/{n_trials}
scoring:
  topics:
{topics_yaml}

task_board_limits:
  nic_rail:
    min_translation: -0.0215
    max_translation: 0.0234
  sc_rail:
    min_translation: -0.06
    max_translation: 0.055
  mount_rail:
    min_translation: -0.09425
    max_translation: 0.09425

trials:
"""

    robot_section = textwrap.dedent("""\
        robot:
          home_joint_positions:
            shoulder_pan_joint: -0.1597
            shoulder_lift_joint: -1.3542
            elbow_joint: -1.6648
            wrist_1_joint: -1.6933
            wrist_2_joint: 1.5710
            wrist_3_joint: 1.4110
    """)

    return header + "\n".join(trial_lines) + "\n" + robot_section, distractor_count


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trials", type=int, default=50)
    parser.add_argument("--seed", type=int, default=99999)
    parser.add_argument("--distractor-prob", type=float, default=0.5,
                        help="Probability that a trial has a distractor (default 0.5).")
    parser.add_argument("--output", type=str,
                        default="aic_engine/config/eval_sc_mixed_50.yaml")
    parser.add_argument("--collection", action="store_true",
                        help="Include image_small topics for bag recording.")
    args = parser.parse_args()

    if not 0.0 <= args.distractor_prob <= 1.0:
        parser.error("--distractor-prob must be in [0.0, 1.0]")

    config_str, distractor_count = generate_config(
        args.trials, args.seed, args.distractor_prob, collection=args.collection,
    )
    Path(args.output).write_text(config_str)
    mode = "collection" if args.collection else "eval"
    print(f"Generated {args.trials} SC-mixed-distractor {mode} trials → {args.output}")
    print(f"  distractor trials: {distractor_count}/{args.trials} "
          f"(target: {args.distractor_prob:.0%})")


if __name__ == "__main__":
    main()
