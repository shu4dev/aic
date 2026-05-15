#!/usr/bin/env python3
"""Lambda Cloud fleet driver for AIC parallel data collection.

Subcommands:
    types       List instance types with current capacity (sanity check).
    launch      Provision N instances; templated bootstrap runs on each.
    wait        Block until every instance is `active` + SSH-reachable
                + has signalled .bootstrap_done.
    run         scp a per-shard part_file to each box, kick off
                run_parallel_multi.sh in the background.
    status      Bag count per box (one-shot snapshot).
    poll        Like status, but loops; exits 0 when every box's runner
                process exits. Use this in a Make target before collect.
    collect     rsync each box's aic_results_* → ./fleet_results/box_<i>/.
    terminate   Terminate every instance recorded in fleet state.
                Idempotent. *Run this even if other commands errored* —
                Lambda bills per minute and a forgotten 8×H100 is ~$100/day.

Fleet state lives in ~/.lambda_fleet/fleet.json. One fleet at a time per
user; if you need concurrent fleets, set LAMBDA_FLEET_DIR.

Required env: LAMBDA_API_KEY  (Bearer token from cloud.lambdalabs.com/api-keys)
Optional env: GHCR_TOKEN      (substituted into bootstrap.sh; required for
                               the worker boxes to pull the private
                               aic-eval image)
              LAMBDA_FLEET_DIR  (overrides ~/.lambda_fleet)

Safety: the script intentionally does NOT auto-terminate on errors. The
caller (Makefile, CI, you) decides. We register an atexit hook that
warns loudly if you exit without terminating, so a forgotten Ctrl-C
during launch won't silently leak instances.
"""

import argparse
import atexit
import json
import os
import pathlib
import shutil
import subprocess
import sys
import time
from typing import Any

import requests

API = "https://cloud.lambdalabs.com/api/v1"


# ── State management ────────────────────────────────────────────────────


def state_dir() -> pathlib.Path:
    d = pathlib.Path(os.environ.get("LAMBDA_FLEET_DIR",
                                    str(pathlib.Path.home() / ".lambda_fleet")))
    d.mkdir(parents=True, exist_ok=True)
    return d


def state_file() -> pathlib.Path:
    return state_dir() / "fleet.json"


def save_state(state: dict[str, Any]) -> None:
    state_file().write_text(json.dumps(state, indent=2))


def load_state() -> dict[str, Any]:
    f = state_file()
    if not f.exists():
        sys.exit(f"no fleet state at {f} — run `launch` first")
    return json.loads(f.read_text())


# ── HTTP helpers ────────────────────────────────────────────────────────


def headers() -> dict[str, str]:
    key = os.environ.get("LAMBDA_API_KEY")
    if not key:
        sys.exit("error: LAMBDA_API_KEY not set\n"
                 "Get a key from https://cloud.lambdalabs.com/api-keys")
    return {"Authorization": f"Bearer {key}"}


def api_get(path: str) -> Any:
    r = requests.get(f"{API}{path}", headers=headers(), timeout=30)
    if not r.ok:
        sys.exit(f"GET {path} failed: {r.status_code} {r.text}")
    return r.json().get("data")


def api_post(path: str, payload: dict[str, Any]) -> Any:
    r = requests.post(f"{API}{path}", headers=headers(), json=payload, timeout=30)
    if not r.ok:
        sys.exit(f"POST {path} failed: {r.status_code} {r.text}")
    return r.json().get("data")


# ── SSH wrapper ────────────────────────────────────────────────────────


SSH_OPTS = [
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "UserKnownHostsFile=/dev/null",
    "-o", "LogLevel=ERROR",
    "-o", "ServerAliveInterval=30",
    "-o", "ConnectTimeout=10",
]


def ssh(ip: str, command: str, check: bool = True,
        capture: bool = False) -> subprocess.CompletedProcess:
    """Run a remote shell command via ssh. Quote `command` yourself — we
    pass it through unchanged so heredocs / pipelines work."""
    return subprocess.run(
        ["ssh", *SSH_OPTS, f"ubuntu@{ip}", command],
        check=check, capture_output=capture, text=True,
    )


def scp_to(ip: str, src: str, dst: str) -> None:
    subprocess.run(["scp", *SSH_OPTS, src, f"ubuntu@{ip}:{dst}"], check=True)


# ── Subcommands ────────────────────────────────────────────────────────


def cmd_types(_args: argparse.Namespace) -> None:
    """List instance types with regions that currently have capacity."""
    data = api_get("/instance-types")
    rows: list[tuple[str, float, list[str]]] = []
    for name, info in (data or {}).items():
        regions = [r["name"] for r in info.get("regions_with_capacity_available", [])]
        if not regions:
            continue
        price = info["instance_type"]["price_cents_per_hour"] / 100.0
        rows.append((name, price, regions))
    rows.sort(key=lambda r: r[1])
    for name, price, regions in rows:
        print(f"{name:30s} ${price:5.2f}/hr  {regions}")
    if not rows:
        print("no types have capacity right now")


def cmd_images(args: argparse.Namespace) -> None:
    """List image families available in a region. Use the printed `family`
    value as --image-family on `launch` (or IMAGE_FAMILY=... via the
    Makefile). We default to Lambda Stack 24.04 because scripts/lambda_bootstrap.sh
    requires Ubuntu Noble; this command is how you confirm the slug or
    discover the right one if Lambda renames the family."""
    data = api_get("/images")
    # /images returns a flat list of image objects, each tagged with region.
    # We dedupe by (family, region) so the same family appearing in N regions
    # collapses into one row with all regions listed.
    seen: dict[str, dict[str, Any]] = {}
    for img in data or []:
        family = img.get("family") or img.get("name") or "<unknown>"
        # /images schema is ambiguous about region — it appears as either a
        # bare string ("us-west-2") or an object ({"name": "us-west-2", ...})
        # in different docs. Handle both so we don't crash.
        region_field = img.get("region")
        if isinstance(region_field, dict):
            region = region_field.get("name", "?")
        elif isinstance(region_field, str):
            region = region_field
        else:
            region = "?"
        if args.region and region != args.region:
            continue
        entry = seen.setdefault(family, {
            "description": img.get("description", ""),
            "version": img.get("version", ""),
            "architecture": img.get("architecture", ""),
            "regions": set(),
        })
        entry["regions"].add(region)
    if not seen:
        print(f"no images returned for region={args.region!r}")
        return
    for family in sorted(seen):
        e = seen[family]
        regions = ",".join(sorted(e["regions"]))
        meta_bits = [b for b in (e["version"], e["architecture"], e["description"]) if b]
        meta = " — " + " | ".join(meta_bits) if meta_bits else ""
        print(f"{family:35s} [{regions}]{meta}")


def _render_bootstrap(path: pathlib.Path) -> str:
    """Substitute {{GHCR_TOKEN}} (and any future placeholders) in the
    bootstrap script. We do this client-side so the token never lives in
    the script on disk; it's injected directly into the launch payload."""
    body = path.read_text()
    ghcr = os.environ.get("GHCR_TOKEN", "")
    if not ghcr:
        print("warning: GHCR_TOKEN not set — bootstrap will skip "
              "`docker login ghcr.io` and rely on cached image, which "
              "fresh boxes won't have. Pull will fail.", file=sys.stderr)
    return body.replace("{{GHCR_TOKEN}}", ghcr)


def cmd_launch(args: argparse.Namespace) -> None:
    """Launch N instances. Persists fleet IDs immediately so a Ctrl-C
    later still lets `terminate` find them."""
    if state_file().exists():
        sys.exit(f"fleet state already exists at {state_file()} — "
                 f"run `terminate` first or remove it manually")

    bootstrap = _render_bootstrap(pathlib.Path(args.bootstrap))
    payload: dict[str, Any] = {
        "region_name": args.region,
        "instance_type_name": args.instance_type,
        "ssh_key_names": [args.ssh_key],
        "quantity": args.num,
        "user_data": bootstrap,
    }
    # Pin the OS image. Required because Lambda's API default is not stable
    # across (region, instance_type) combinations — we've seen it boot jammy
    # (22.04) on gpu_1x_a100_sxm4/us-west-2, which the bootstrap's Noble
    # precondition then rejects. Empty string opts back into Lambda's default.
    if args.image_family:
        payload["image"] = {"family": args.image_family}
    if args.file_system:
        payload["file_system_names"] = [args.file_system]
    if args.name:
        payload["name"] = args.name

    data = api_post("/instance-operations/launch", payload)
    ids = data["instance_ids"]
    state = {
        "instance_ids": ids,
        "instances": [{"id": i, "ip": None, "status": "pending"} for i in ids],
        "launch": {
            "region": args.region,
            "instance_type": args.instance_type,
            "num": args.num,
            "time": time.time(),
        },
    }
    save_state(state)
    print(f"launched {len(ids)} instances:")
    for i in ids:
        print(f"  {i}")
    print("\nrun `wait` next; remember to `terminate` when you're done.")


def _refresh_instance_status() -> dict[str, dict[str, Any]]:
    """Return id → instance-info from the API for our tracked instances."""
    state = load_state()
    target = set(state["instance_ids"])
    by_id = {i["id"]: i for i in (api_get("/instances") or [])}
    return {i: by_id.get(i, {"id": i, "status": "unknown"}) for i in target}


def cmd_wait(args: argparse.Namespace) -> None:
    """Poll until every instance is `active` with an IP AND the bootstrap
    script's marker file exists. The marker check is what tells us the
    pixi install + image pull + setup_workers actually finished."""
    state = load_state()
    target = set(state["instance_ids"])
    deadline = time.time() + args.timeout

    instances_active: list[dict[str, Any]] = []
    while time.time() < deadline:
        live = _refresh_instance_status()
        instances_active = [i for i in live.values()
                            if i.get("status") == "active" and i.get("ip")]
        print(f"  {len(instances_active)}/{len(target)} active "
              f"(elapsed {int(time.time() - state['launch']['time'])}s)")
        if len(instances_active) == len(target):
            break
        time.sleep(args.poll)
    else:
        sys.exit(f"timeout: only {len(instances_active)}/{len(target)} "
                 f"reached active within {args.timeout}s")

    # Persist IPs so subsequent commands don't need to re-hit the API.
    state["instances"] = [{"id": i["id"], "ip": i["ip"], "status": i["status"]}
                          for i in instances_active]
    save_state(state)

    # Now wait for bootstrap to finish on each box.
    print("\nwaiting for bootstrap (~25 min on cold boxes)...")
    pending = {i["ip"]: i["id"] for i in instances_active}
    while pending and time.time() < deadline:
        for ip in list(pending):
            r = ssh(ip, "test -f ~/.bootstrap_done && echo READY",
                    check=False, capture=True)
            if r.returncode == 0 and "READY" in (r.stdout or ""):
                print(f"  ✓ {pending[ip]} ({ip}) bootstrap done")
                del pending[ip]
        if pending:
            time.sleep(args.poll)
    if pending:
        sys.exit(f"timeout: {len(pending)} boxes never finished bootstrap")
    print(f"\nall {len(instances_active)} boxes ready.")


def cmd_run(args: argparse.Namespace) -> None:
    """Split the input config N ways and kick off run_parallel_multi.sh
    in the background on each box."""
    state = load_state()
    instances = state["instances"]
    n = len(instances)
    if n == 0:
        sys.exit("no active instances")

    # Split the input config locally, then ship one shard per box.
    shard_dir = pathlib.Path("/tmp/lambda_fleet_shards")
    shutil.rmtree(shard_dir, ignore_errors=True)
    shard_dir.mkdir()
    split_script = pathlib.Path(__file__).parent / "split_trials.py"
    print(f"splitting {args.config} into {n} shards...")
    subprocess.run(
        ["pixi", "run", "python", str(split_script),
         "--input", args.config,
         "--output-dir", str(shard_dir),
         "--num-parts", str(n)],
        check=True,
    )

    # Per-box: scp shard, then kick off run_parallel_multi via nohup. The
    # `&` + nohup is critical — without it the ssh would block until the
    # run finishes (hours).
    for idx, inst in enumerate(instances):
        ip = inst["ip"]
        shard = shard_dir / f"train_part_{idx}.yaml"
        scp_to(ip, str(shard), "/home/ubuntu/shard.yaml")
        remote = (
            "cd ~/ws_aic/src/aic && "
            f"NUM_WORKERS={args.workers_per_box} "
            f"POLICY={args.policy} "
            f"INPUT_CONFIG=/home/ubuntu/shard.yaml "
            f"STAGGER_SECS={args.stagger} "
            f"CHUNK_TRIALS={args.chunk} "
            f"MAX_CHUNK_RETRIES={args.max_chunk_retries} "
            "nohup bash scripts/run_parallel_multi.sh "
            "> /home/ubuntu/run.log 2>&1 &"
        )
        ssh(ip, remote)
        print(f"  ✓ kicked off on {ip} (box {idx}, shard part_{idx}.yaml)")

    state["run"] = {
        "started": time.time(),
        "config": args.config,
        "policy": args.policy,
        "workers_per_box": args.workers_per_box,
    }
    save_state(state)
    print(f"\n{n} boxes running. Use `status` for snapshot, `poll` to "
          f"block until all finish.")


def _bag_count(ip: str) -> int:
    r = ssh(ip,
            "find /home/ubuntu/aic-data -maxdepth 4 -type d "
            "-name 'bag_trial_*' 2>/dev/null | wc -l",
            check=False, capture=True)
    try:
        return int((r.stdout or "0").strip())
    except ValueError:
        return -1


def _is_running(ip: str) -> bool:
    r = ssh(ip, "pgrep -f run_parallel_multi.sh >/dev/null && echo YES",
            check=False, capture=True)
    return "YES" in (r.stdout or "")


def cmd_status(_args: argparse.Namespace) -> None:
    """Snapshot of bag count + alive/dead per box."""
    state = load_state()
    total = 0
    for idx, inst in enumerate(state["instances"]):
        ip = inst["ip"]
        bags = _bag_count(ip)
        alive = "running" if _is_running(ip) else "exited"
        print(f"  box {idx} {ip}: {bags:5d} bags  [{alive}]")
        total += max(0, bags)
    if "run" in state:
        elapsed = int(time.time() - state["run"]["started"])
        rate = total / max(1, elapsed / 60)
        print(f"\ntotal: {total} bags, {elapsed//60} min elapsed, "
              f"{rate:.2f} bag/min aggregate")


def cmd_poll(args: argparse.Namespace) -> None:
    """Block until every box's run_parallel_multi.sh exits."""
    state = load_state()
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        alive = sum(1 for inst in state["instances"] if _is_running(inst["ip"]))
        total = sum(max(0, _bag_count(inst["ip"])) for inst in state["instances"])
        print(f"  {alive}/{len(state['instances'])} boxes still running, "
              f"{total} bags collected")
        if alive == 0:
            print("all boxes done")
            return
        time.sleep(args.poll)
    sys.exit(f"timeout after {args.timeout}s with boxes still running")


def cmd_collect(args: argparse.Namespace) -> None:
    """rsync aic_results_* from every box to ./fleet_results/box_<i>/."""
    state = load_state()
    out = pathlib.Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    for idx, inst in enumerate(state["instances"]):
        ip = inst["ip"]
        dest = out / f"box_{idx}"
        dest.mkdir(exist_ok=True)
        print(f"  rsync {ip} → {dest}")
        # Trailing slash on src/aic-data/ pulls contents, not the dir.
        ssh_cmd = "ssh " + " ".join(SSH_OPTS)
        subprocess.run(
            ["rsync", "-az", "--info=progress2",
             "-e", ssh_cmd,
             f"ubuntu@{ip}:/home/ubuntu/aic-data/",
             str(dest) + "/"],
            check=True,
        )
    print(f"\nresults in {out.resolve()}")


def cmd_terminate(_args: argparse.Namespace) -> None:
    """Terminate all tracked instances. Idempotent. Always safe to run."""
    if not state_file().exists():
        print("no fleet state — nothing to terminate")
        return
    state = load_state()
    ids = state["instance_ids"]
    print(f"terminating {len(ids)} instances...")
    api_post("/instance-operations/terminate", {"instance_ids": ids})
    state_file().unlink()
    print("terminated. fleet state cleared.")


# ── atexit safety net ──────────────────────────────────────────────────


def _warn_if_leaked() -> None:
    if state_file().exists() and sys.exc_info()[0] is not None:
        print("\n" + "!" * 70, file=sys.stderr)
        print("WARNING: fleet state still exists at", state_file(),
              file=sys.stderr)
        print("If you launched instances, run `terminate` NOW or you'll be "
              "billed for them.", file=sys.stderr)
        print("!" * 70, file=sys.stderr)


atexit.register(_warn_if_leaked)


# ── argparse ────────────────────────────────────────────────────────────


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("types", help="list instance types with capacity").set_defaults(fn=cmd_types)

    pi = sub.add_parser("images", help="list available image families")
    pi.add_argument("--region", default="us-west-2",
                    help="filter to one region (default us-west-2); pass '' for all")
    pi.set_defaults(fn=cmd_images)

    pl = sub.add_parser("launch", help="launch N instances")
    pl.add_argument("--num", type=int, default=4, help="number of instances (default 4)")
    pl.add_argument("--region", default="us-west-2",
                    help="Lambda region (default us-west-2)")
    pl.add_argument("--instance-type", default="gpu_1x_a100_sxm4")
    pl.add_argument("--ssh-key", required=True,
                    help="name of an SSH key already registered with Lambda")
    pl.add_argument("--image-family", default="lambda-stack-24-04",
                    help="Lambda image family to boot (run `images` to list). "
                         "Default pins to Lambda Stack 24.04 (Ubuntu Noble), which "
                         "scripts/lambda_bootstrap.sh requires. Pass '' to fall "
                         "back to Lambda's API default.")
    pl.add_argument("--file-system", default=None,
                    help="name of a Lambda persistent FS to attach (optional)")
    pl.add_argument("--bootstrap",
                    default=str(pathlib.Path(__file__).parent / "lambda_bootstrap.sh"),
                    help="path to bootstrap.sh (templated server-side)")
    pl.add_argument("--name", default=None, help="optional human label")
    pl.set_defaults(fn=cmd_launch)

    pw = sub.add_parser("wait", help="block until instances are bootstrapped")
    pw.add_argument("--poll", type=int, default=15)
    pw.add_argument("--timeout", type=int, default=2400, help="max seconds (default 40 min)")
    pw.set_defaults(fn=cmd_wait)

    pr = sub.add_parser("run", help="distribute shards + start collection")
    pr.add_argument("--config", required=True, help="trial config to split")
    pr.add_argument("--workers-per-box", type=int, default=1,
                    help="NUM_WORKERS inside run_parallel_multi.sh. Default 1 "
                         "= fully serial per box (one container processes the "
                         "whole shard in CHUNK_TRIALS-sized chunks, recycling "
                         "between chunks). Bump for parallel workers per box.")
    pr.add_argument("--policy", default="aic_example_policies.ros.SCCheatCode")
    pr.add_argument("--stagger", type=int, default=60)
    pr.add_argument("--chunk", type=int, default=10,
                    help="CHUNK_TRIALS — trials per worker before its container is "
                         "rm'd + recreated. Recycling between chunks prevents the "
                         "heap corruption seen after ~70 cumulative trials in one "
                         "container. Pass 0 to disable chunking.")
    pr.add_argument("--max-chunk-retries", type=int, default=3,
                    help="MAX_CHUNK_RETRIES — how many times run_parallel_multi.sh "
                         "retries a failed chunk (recreate container + rerun) "
                         "before giving up on it.")
    pr.set_defaults(fn=cmd_run)

    sub.add_parser("status", help="bag count snapshot").set_defaults(fn=cmd_status)

    pp = sub.add_parser("poll", help="block until all boxes' runs finish")
    pp.add_argument("--poll", type=int, default=120)
    pp.add_argument("--timeout", type=int, default=72000, help="max seconds (default 20 hr)")
    pp.set_defaults(fn=cmd_poll)

    pc = sub.add_parser("collect", help="rsync results from every box")
    pc.add_argument("--output", default="./fleet_results")
    pc.set_defaults(fn=cmd_collect)

    sub.add_parser("terminate", help="terminate all tracked instances").set_defaults(fn=cmd_terminate)

    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
