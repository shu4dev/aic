#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
"""Hybrid policy: ACT for approach, classical force-feedback descent + spiral search.

Design intent: the trained ACT policy reliably approaches the port but was not
trained to descend and insert. Rather than retraining, this policy runs ACT
inference until a contact force is detected on the FT sensor, then hands off to
a classical (model-free) controller for descent and insertion.

Phases:
  1. ACT approach — runs RunACT inference per-step; monitors smoothed |F_z|
     against a baseline measured in the first 10 iterations. Handoff when
     df > HANDOFF_FORCE_THRESHOLD for HANDOFF_CONFIRM_FRAMES consecutive frames,
     or when HANDOFF_TIMEOUT_S sim seconds have elapsed.

  2. Force-controlled descent + spiral search — anchored at the TCP pose
     captured at handoff (NOT a TF lookup of the plug, so the controller is
     transferable to real hardware). Per-connector params (SFP / SC) lifted
     from ImprovedCheatCode. Spiral overlay engages when descent stalls.
     Jam detection retreats and aborts after MAX_JAMS consecutive jams.

  3. Stabilize + joint-space hold — mirrors ImprovedCheatCode's end-of-trial
     pattern to prevent drift before lifecycle deactivation.
"""

import math
import os
from collections import deque

import numpy as np
from geometry_msgs.msg import Point, Pose
from trajectory_msgs.msg import JointTrajectoryPoint

from aic_control_interfaces.msg import (
    JointMotionUpdate,
    TrajectoryGenerationMode,
)
from aic_model.policy import (
    GetObservationCallback,
    MoveRobotCallback,
    Policy,
    SendFeedbackCallback,
)
from aic_task_interfaces.msg import Task

from .RunACT import RunACT


# Per-connector descent params (lifted from ImprovedCheatCode.CONNECTOR_PARAMS).
# max_depth here is the descent distance from the handoff TCP (positive).
CONNECTOR_PARAMS = {
    "sfp": {
        "max_depth": 0.022,
        "coarse_step": 0.0012,
        "fine_step": 0.0003,
        "fine_threshold": 0.01,
        "force_limit": 15.0,
        "force_ref": 4.0,
        "force_soft": 8.0,
    },
    "sc": {
        "max_depth": 0.035,
        "coarse_step": 0.0005,
        "fine_step": 0.0003,
        "fine_threshold": 0.008,
        "force_limit": 12.0,
        "force_ref": 3.5,
        "force_soft": 7.0,
    },
}
DEFAULT_PARAMS = CONNECTOR_PARAMS["sfp"]


class RunACTThenInsert(Policy):
    def __init__(self, parent_node):
        super().__init__(parent_node)
        # Composition: instantiate RunACT internally. This triggers ACT's
        # heavy imports (torch, lerobot, cv2) and loads the checkpoint exactly
        # once, before aic_engine activates the lifecycle.
        self._act = RunACT(parent_node)

        # Phase-1 (handoff) knobs. The approach is considered "done" when the
        # TCP has barely moved over a sliding sim-time window — i.e., ACT has
        # finished its trajectory and is just holding pose. A force-only
        # trigger fires too early because ACT itself exerts force during its
        # own descent toward the port.
        self._settled_window_s = float(os.environ.get("HANDOFF_SETTLED_WINDOW_S", "1.5"))
        self._settled_disp_m = float(os.environ.get("HANDOFF_SETTLED_DISPLACEMENT_M", "0.001"))
        self._min_approach_s = float(os.environ.get("HANDOFF_MIN_APPROACH_S", "3.0"))
        self._handoff_timeout_s = float(os.environ.get("HANDOFF_TIMEOUT_S", "60.0"))

        # Phase-2 (descent) knobs
        self._descent_stiffness_xy = float(os.environ.get("DESCENT_STIFFNESS_XY", "400.0"))
        self._spiral_r0 = float(os.environ.get("SPIRAL_R0", "0.0005"))
        self._spiral_rate = float(os.environ.get("SPIRAL_RATE", "0.0003"))
        self._spiral_freq = float(os.environ.get("SPIRAL_FREQ", "0.5"))
        self._spiral_max_r = float(os.environ.get("SPIRAL_MAX_R", "0.005"))
        self._max_jams = int(os.environ.get("MAX_JAMS", "5"))

        # Phase-2 round-2 additions: cable-settle pause + wrench-based
        # lateral compliance (P-only admittance). Compliance replaces
        # ImprovedCheatCode's plug-TF PI XY-correction; spiral is now gated
        # so it only engages when the contact gives no directional force.
        self._cable_settle_s = float(os.environ.get("CABLE_SETTLE_S", "1.5"))
        self._k_lateral = float(os.environ.get("K_LATERAL", "0.0005"))
        self._max_lateral_m = float(os.environ.get("MAX_LATERAL_M", "0.005"))
        self._compliance_dead_band_n = float(
            os.environ.get("COMPLIANCE_DEAD_BAND_N", "0.5")
        )
        self._spiral_force_threshold_n = float(
            os.environ.get("SPIRAL_FORCE_THRESHOLD_N", "1.0")
        )

        # Descent progress guard. Without this, the loop can spin forever on
        # a force-oscillation limit cycle (df hovers near F_soft, step sign
        # flips each iteration, descent_z never grows, jam threshold never
        # reached). Track the best descent_z achieved and abort if no new
        # high in DESCENT_STALL_TIMEOUT_S sim seconds.
        self._descent_stall_timeout_s = float(
            os.environ.get("DESCENT_STALL_TIMEOUT_S", "10.0")
        )
        self._progress_epsilon_m = float(
            os.environ.get("PROGRESS_EPSILON_M", "0.0002")
        )

        self.get_logger().info(
            f"[RunACTThenInsert] init: settled_window={self._settled_window_s}s "
            f"settled_disp={self._settled_disp_m*1000:.2f}mm "
            f"min_approach={self._min_approach_s}s "
            f"timeout={self._handoff_timeout_s}s max_jams={self._max_jams} "
            f"cable_settle={self._cable_settle_s}s "
            f"k_lat={self._k_lateral} max_lat={self._max_lateral_m*1000:.1f}mm "
            f"dead_band={self._compliance_dead_band_n}N "
            f"spiral_force_thresh={self._spiral_force_threshold_n}N "
            f"stall_timeout={self._descent_stall_timeout_s}s "
            f"progress_eps={self._progress_epsilon_m*1000:.2f}mm"
        )

    # ---------------------------------------------------------------------
    # Force / wrench helpers
    # ---------------------------------------------------------------------
    def _read_wrench(self, obs_msg):
        """Smoothed (Fx, Fy, Fz, Tx, Ty, Tz) at this frame's center-camera
        timestamp. Falls back to the single-sample wrist wrench if the
        smoother buffer is empty."""
        stamp = obs_msg.center_image.header.stamp
        ref_time = stamp.sec + stamp.nanosec * 1e-9
        smoothed = self._act._wrench_smoother.smoothed(
            ref_time,
            window_s=self._act._wrench_window_s,
            mode=self._act._wrench_mode,
        )
        if smoothed is not None:
            return tuple(float(x) for x in smoothed)
        w = obs_msg.wrist_wrench.wrench
        return (
            float(w.force.x), float(w.force.y), float(w.force.z),
            float(w.torque.x), float(w.torque.y), float(w.torque.z),
        )

    # ---------------------------------------------------------------------
    # Phase 1 — ACT approach until trajectory completes (TCP stillness)
    # ---------------------------------------------------------------------
    def _run_act_until_settled(self, task, get_observation, move_robot, send_feedback):
        """Run ACT inference loop until the TCP stops moving.

        The trained ACT approach descends to just above the port and holds.
        We hand off when ACT has finished that trajectory — measured as TCP
        displacement across a sliding sim-time window. Wrench is logged for
        diagnostics but is not the trigger: a force-based handoff fires too
        early because ACT itself exerts force during its own descent.

        Returns dict {tcp, baseline_fz, contact_fz, reason} or None if no
        observation ever arrived.
        """
        # Mirror RunACT.insert_cable's loop scaffolding so the ACT behaviour
        # during this phase is identical to running RunACT standalone.
        CONTROL_HZ = float(os.environ.get("ACT_CONTROL_HZ", "20.0"))
        CONTROL_DT = 1.0 / CONTROL_HZ
        RESET_EVERY = int(os.environ.get("ACT_RESET_EVERY", "10"))

        te_coeff_str = os.environ.get("ACT_TEMPORAL_ENSEMBLE", "0.1")
        use_temporal_ensemble = te_coeff_str not in ("", "off", "0")
        if use_temporal_ensemble:
            te_coeff = float(te_coeff_str)
            if self._act.policy.config.temporal_ensemble_coeff is None:
                self._act.policy.config.temporal_ensemble_coeff = te_coeff
                self._act.policy.config.n_action_steps = 1
                from lerobot.policies.act.modeling_act import ACTTemporalEnsembler
                self._act.policy.temporal_ensembler = ACTTemporalEnsembler(
                    te_coeff, self._act.policy.config.chunk_size
                )

        # Reset ACT per-trial state (mirrors what RunACT.insert_cable does).
        self._act.policy.reset()
        self._act._port_entrance_baselink = None
        self._act._current_task = task

        # Sliding window of (sim_time_s, x, y, z) for stillness detection.
        tcp_history = deque()
        baseline_samples = []  # for diagnostic fz baseline only
        baseline_fz = None
        BASELINE_FRAMES = 10
        handoff_tcp = None
        reason = None
        iter_count = 0
        last_tcp = None

        send_feedback("Approaching (ACT)...")
        self.get_logger().info(
            f"[approach] starting ACT phase, "
            f"settle window={self._settled_window_s}s "
            f"disp_thresh={self._settled_disp_m*1000:.2f}mm "
            f"min_approach={self._min_approach_s}s "
            f"timeout={self._handoff_timeout_s}s "
            f"te={te_coeff_str if use_temporal_ensemble else 'off'}"
        )

        start_sim = self.time_now()
        while True:
            sim_elapsed = (self.time_now() - start_sim).nanoseconds * 1e-9
            if sim_elapsed >= self._handoff_timeout_s:
                reason = "timeout"
                self.get_logger().warn(
                    f"[handoff] timeout at sim={sim_elapsed:.1f}s "
                    f"(TCP never settled below {self._settled_disp_m*1000:.2f}mm)"
                )
                break

            loop_start = self.time_now()
            obs = get_observation()
            if obs is None:
                self.get_logger().info("[approach] no observation, aborting")
                return None
            last_tcp = obs.controller_state.tcp_pose

            # Periodic chunk reset (only when TE is off)
            if (not use_temporal_ensemble
                    and iter_count > 0
                    and iter_count % RESET_EVERY == 0):
                self._act.policy.reset()

            # ACT inference + dispatch (move_robot called inside _act_step)
            self._act._act_step(obs, move_robot)

            # --- Diagnostic: track force baseline (not used as a trigger) ---
            fz = self._read_wrench(obs)[2]
            if len(baseline_samples) < BASELINE_FRAMES:
                baseline_samples.append(fz)
                if len(baseline_samples) == BASELINE_FRAMES:
                    baseline_fz = sum(baseline_samples) / BASELINE_FRAMES
                    self.get_logger().info(
                        f"[approach] baseline_fz={baseline_fz:.2f}N (n={BASELINE_FRAMES})"
                    )

            # --- Primary trigger: TCP stillness over a sliding sim-time window ---
            tcp_history.append(
                (sim_elapsed, last_tcp.position.x, last_tcp.position.y, last_tcp.position.z)
            )
            # Drop entries older than the settle window
            while tcp_history and (sim_elapsed - tcp_history[0][0]) > self._settled_window_s:
                tcp_history.popleft()

            # Only check stillness once we have a full window and ACT has had
            # time to start moving (min_approach guard avoids firing on the
            # initial pre-motion frames).
            disp = None
            if (sim_elapsed >= self._min_approach_s
                    and len(tcp_history) >= 2
                    and (tcp_history[-1][0] - tcp_history[0][0]) >= self._settled_window_s * 0.9):
                xs = [e[1] for e in tcp_history]
                ys = [e[2] for e in tcp_history]
                zs = [e[3] for e in tcp_history]
                disp = max(max(xs) - min(xs),
                           max(ys) - min(ys),
                           max(zs) - min(zs))
                if disp < self._settled_disp_m:
                    handoff_tcp = obs.controller_state.tcp_pose
                    reason = "settled"
                    self.get_logger().info(
                        f"[handoff] TCP settled at iter={iter_count} "
                        f"disp={disp*1000:.2f}mm over {self._settled_window_s}s "
                        f"fz={fz:.2f}N "
                        f"tcp=({handoff_tcp.position.x:.3f},"
                        f"{handoff_tcp.position.y:.3f},"
                        f"{handoff_tcp.position.z:.3f})"
                    )
                    break

            if iter_count % 25 == 0 and baseline_fz is not None:
                disp_str = f"{disp*1000:.2f}mm" if disp is not None else "n/a"
                self.get_logger().info(
                    f"[approach] iter={iter_count} sim={sim_elapsed:.1f}s "
                    f"disp_{self._settled_window_s:g}s={disp_str} "
                    f"fz={fz:.2f}N df={abs(fz - baseline_fz):.2f}N"
                )

            send_feedback("approaching...")
            iter_count += 1
            elapsed = (self.time_now() - loop_start).nanoseconds * 1e-9
            self.sleep_for(max(0.0, CONTROL_DT - elapsed))

        # Timeout path: anchor on the last observed TCP.
        if handoff_tcp is None:
            handoff_tcp = last_tcp
            if handoff_tcp is None:
                return None

        return {
            "tcp": handoff_tcp,
            "baseline_fz": baseline_fz if baseline_fz is not None else 0.0,
            "contact_fz": None,
            "reason": reason or "unknown",
        }

    # ---------------------------------------------------------------------
    # Phase 1b — Cable-settle pause (between handoff and descent)
    # ---------------------------------------------------------------------
    def _cable_settle_pause(self, handoff_state, move_robot):
        """Hold the handoff TCP pose for CABLE_SETTLE_S seconds.

        Mirrors ImprovedCheatCode's settle pause (lines 297-318): ACT's
        approach can excite the cable's swing momentum (20-segment ball-joint
        model, damping=0.2, zero spring stiffness); if descent begins
        immediately, that residual swing drags the plug laterally. A short
        firm hold dissipates it.
        """
        if self._cable_settle_s <= 0.0:
            return
        anchor = handoff_state["tcp"]
        stiffness = [self._descent_stiffness_xy] * 3 + [150.0, 150.0, 150.0]
        damping = [200.0, 200.0, 200.0, 60.0, 60.0, 60.0]
        n_iters = max(1, int(self._cable_settle_s / 0.04))
        self.get_logger().info(
            f"[settle] holding handoff pose for {self._cable_settle_s:.2f}s "
            f"({n_iters} iters) "
            f"tcp=({anchor.position.x:.3f},{anchor.position.y:.3f},{anchor.position.z:.3f})"
        )
        for _ in range(n_iters):
            self.set_pose_target(
                move_robot=move_robot,
                pose=anchor,
                stiffness=stiffness,
                damping=damping,
            )
            self.sleep_for(0.04)

    # ---------------------------------------------------------------------
    # Phase 2 — Force-controlled descent + lateral compliance + spiral search
    # ---------------------------------------------------------------------
    def _scripted_descent(self, handoff_state, task, get_observation,
                          move_robot, send_feedback):
        """Descent anchored on the handoff TCP pose. Uses TCP pose + wrench
        only (no ground-truth TF). Lateral correction is a P-only admittance
        on (Fx, Fy); spiral search engages only when the contact gives no
        directional force. Returns the final descent distance reached.
        """
        params = CONNECTOR_PARAMS.get(task.plug_type, DEFAULT_PARAMS)
        anchor = handoff_state["tcp"]
        baseline_fz = handoff_state["baseline_fz"]

        self.get_logger().info(
            f"[descent] start plug_type={task.plug_type} "
            f"max_depth={params['max_depth']*1000:.1f}mm "
            f"force_ref={params['force_ref']}N force_limit={params['force_limit']}N"
        )
        send_feedback("Descending...")

        DT = 0.04  # 25 Hz — matches ImprovedCheatCode's descent cadence

        descent_z = 0.0
        consecutive_jams = 0
        stall_count = 0
        spiral_active = False
        spiral_t = 0.0
        resume_count = 0
        prev_tcp_z = anchor.position.z
        iter_count = 0
        # Progress guard: track the best actual TCP descent and the sim time
        # of the last "new high" so we can abort if descent stalls without
        # tripping the force_limit-based jam detector.
        best_actual_z = 0.0
        last_progress_t = self.time_now()

        stiffness = [self._descent_stiffness_xy] * 3 + [150.0, 150.0, 150.0]
        damping = [200.0, 200.0, 200.0, 60.0, 60.0, 60.0]

        while True:
            obs = get_observation()
            if obs is None:
                self.get_logger().warn("[descent] no observation, aborting")
                break

            tcp = obs.controller_state.tcp_pose
            fx, fy, fz = self._read_wrench(obs)[:3]
            df = abs(fz - baseline_fz)
            f_xy_mag = math.sqrt(fx * fx + fy * fy)

            # 1. Force-modulated step size (ImprovedCheatCode formula)
            base_step = (params["coarse_step"]
                         if descent_z < params["fine_threshold"]
                         else params["fine_step"])
            F_ref = params["force_ref"]
            F_soft = params["force_soft"]
            if df <= F_ref:
                step = base_step
            elif df < F_soft:
                scale = max(0.1, 1.0 - (df - F_ref) / (F_soft - F_ref))
                step = base_step * scale
            else:
                step = -0.5 * base_step  # back off

            # 2. Actual Z motion since last iteration (positive = descended)
            dz_actual = prev_tcp_z - tcp.position.z
            prev_tcp_z = tcp.position.z

            # 2b. Progress guard. Track the deepest TCP descent reached and
            # abort if no new high is set within DESCENT_STALL_TIMEOUT_S.
            # Catches force-oscillation limit cycles and compliance-masked
            # stalls — neither trips the force_limit-based jam detector.
            actual_descent = anchor.position.z - tcp.position.z
            if actual_descent > best_actual_z + self._progress_epsilon_m:
                best_actual_z = actual_descent
                last_progress_t = self.time_now()
            else:
                stalled_for = (self.time_now() - last_progress_t).nanoseconds * 1e-9
                if stalled_for > self._descent_stall_timeout_s:
                    self.get_logger().warn(
                        f"[descent] no progress for {stalled_for:.1f}s "
                        f"(best_actual={best_actual_z*1000:.2f}mm, "
                        f"commanded_z={descent_z*1000:.2f}mm, "
                        f"df={df:.1f}N), aborting"
                    )
                    break

            # 3. Lateral compliance (P-only admittance on Fx, Fy).
            #    Dead-band suppresses sensor-noise drift. Circular saturation
            #    bounds the offset like ImprovedCheatCode's max_windup.
            if f_xy_mag < self._compliance_dead_band_n:
                comply_dx, comply_dy = 0.0, 0.0
            else:
                db_scale = (f_xy_mag - self._compliance_dead_band_n) / f_xy_mag
                fx_eff = fx * db_scale
                fy_eff = fy * db_scale
                comply_dx = -self._k_lateral * fx_eff
                comply_dy = -self._k_lateral * fy_eff
                cmag = math.sqrt(comply_dx * comply_dx + comply_dy * comply_dy)
                if cmag > self._max_lateral_m:
                    sat = self._max_lateral_m / cmag
                    comply_dx *= sat
                    comply_dy *= sat

            # 4. Stall detection. Spiral is gated on small |F_xy| — when the
            #    contact gives no directional force, compliance has nothing
            #    to act on and blind spiral search is the only fallback.
            stalled = (df > F_ref) and (dz_actual < 0.3 * base_step)
            no_lateral_signal = f_xy_mag < self._spiral_force_threshold_n
            if stalled:
                stall_count += 1
                if (stall_count > 5
                        and not spiral_active
                        and no_lateral_signal):
                    spiral_active = True
                    spiral_t = 0.0
                    self.get_logger().info(
                        f"[descent] spiral engaged at z={descent_z*1000:.2f}mm "
                        f"|F_xy|={f_xy_mag:.2f}N (no directional signal)"
                    )
            else:
                if spiral_active:
                    resume_count += 1
                    if resume_count >= 3:
                        self.get_logger().info(
                            f"[descent] spiral disengaged at z={descent_z*1000:.2f}mm"
                        )
                        spiral_active = False
                        spiral_t = 0.0
                        resume_count = 0
                else:
                    stall_count = max(0, stall_count - 1)

            # 5. Jam detection — retreat and count
            if df > params["force_limit"] and dz_actual < 0.3 * base_step:
                consecutive_jams += 1
                self.get_logger().warn(
                    f"[descent] JAM df={df:.1f}N dz={dz_actual*1000:.3f}mm "
                    f"z={descent_z*1000:.2f}mm jams={consecutive_jams}"
                )
                descent_z = max(0.0, descent_z - 3 * base_step)
                if consecutive_jams >= self._max_jams:
                    self.get_logger().warn("[descent] max jams reached, aborting")
                    break
                self.sleep_for(0.1)
            elif df < params["force_limit"] * 0.3:
                consecutive_jams = max(0, consecutive_jams - 1)

            # 6. Advance descent depth
            descent_z += step
            if descent_z >= params["max_depth"]:
                self.get_logger().info(
                    f"[descent] reached max_depth={params['max_depth']*1000:.1f}mm"
                )
                break
            if descent_z < 0.0:
                descent_z = 0.0

            # 7. Spiral overlay (active only when stalled AND no force signal)
            spiral_dx, spiral_dy, r_now = 0.0, 0.0, 0.0
            if spiral_active:
                spiral_t += DT
                r_now = min(self._spiral_max_r,
                            self._spiral_r0 + self._spiral_rate * spiral_t)
                theta = 2.0 * math.pi * self._spiral_freq * spiral_t
                spiral_dx = r_now * math.cos(theta)
                spiral_dy = r_now * math.sin(theta)

            # 8. Build target pose and dispatch.
            #    target = anchor + lateral_compliance + spiral_overlay - descent_z
            target = Pose(
                position=Point(
                    x=anchor.position.x + comply_dx + spiral_dx,
                    y=anchor.position.y + comply_dy + spiral_dy,
                    z=anchor.position.z - descent_z,
                ),
                orientation=anchor.orientation,
            )
            self.set_pose_target(
                move_robot=move_robot,
                pose=target,
                stiffness=stiffness,
                damping=damping,
            )

            if iter_count % 25 == 0:
                self.get_logger().info(
                    f"[descent] z={descent_z*1000:.2f}mm df={df:.1f}N "
                    f"dz={dz_actual*1000:.3f}mm step={step*1000:.3f}mm "
                    f"|F_xy|={f_xy_mag:.2f}N "
                    f"comply=({comply_dx*1000:+.2f},{comply_dy*1000:+.2f})mm "
                    f"spiral_r={r_now*1000:.2f}mm "
                    f"spiral={'Y' if spiral_active else 'N'} "
                    f"jams={consecutive_jams}"
                )

            send_feedback("inserting...")
            iter_count += 1
            self.sleep_for(DT)

        return descent_z

    # ---------------------------------------------------------------------
    # Phase 3 — Stabilize + joint-space hold (from ImprovedCheatCode end)
    # ---------------------------------------------------------------------
    def _stabilize_and_hold(self, get_observation, move_robot):
        """Hold the final Cartesian pose, then switch to joint-space hold."""
        self.get_logger().info("[stabilize] holding final pose")
        obs = get_observation()
        if obs is None:
            return
        final_pose = obs.controller_state.tcp_pose
        for _ in range(20):
            self.set_pose_target(move_robot=move_robot, pose=final_pose)
            self.sleep_for(0.04)

        obs = get_observation()
        if (obs is not None and obs.joint_states is not None
                and len(obs.joint_states.position) >= 6):
            joints = list(obs.joint_states.position[:6])
            self.get_logger().info(
                f"[end_hold] freezing arm at "
                f"joints=[{', '.join(f'{j:.3f}' for j in joints)}]"
            )
            hold_cmd = JointMotionUpdate(
                target_state=JointTrajectoryPoint(positions=joints),
                target_stiffness=[500.0, 500.0, 500.0, 200.0, 200.0, 200.0],
                target_damping=[40.0, 40.0, 40.0, 15.0, 15.0, 15.0],
                trajectory_generation_mode=TrajectoryGenerationMode(
                    mode=TrajectoryGenerationMode.MODE_POSITION,
                ),
            )
            for _ in range(15):
                move_robot(joint_motion_update=hold_cmd)
                self.sleep_for(0.04)
        else:
            self.get_logger().warn("[end_hold] observation unavailable, skipping")

    # ---------------------------------------------------------------------
    # Entry point
    # ---------------------------------------------------------------------
    def insert_cable(
        self,
        task: Task,
        get_observation: GetObservationCallback,
        move_robot: MoveRobotCallback,
        send_feedback: SendFeedbackCallback,
        **kwargs,
    ):
        self.get_logger().info(
            f"RunACTThenInsert.insert_cable() task={task}"
        )

        handoff = self._run_act_until_settled(
            task, get_observation, move_robot, send_feedback
        )
        if handoff is None:
            self.get_logger().warn("ACT phase aborted (no observation)")
            return False

        self._cable_settle_pause(handoff, move_robot)

        self._scripted_descent(
            handoff, task, get_observation, move_robot, send_feedback
        )
        self._stabilize_and_hold(get_observation, move_robot)

        self.get_logger().info("RunACTThenInsert.insert_cable() exiting...")
        return True
