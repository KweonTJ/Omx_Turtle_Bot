#!/usr/bin/env python3
# flake8: noqa
"""Run matched-seed MuJoCo trials for reference-only and residual PPO control."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import sys
import time
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", "/tmp/omx_rl_matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats
from stable_baselines3 import PPO
import torch
import yaml


CONTROLLERS = ("mp_reference_proxy", "rl_residual")
LABELS = {
    "mp_reference_proxy": "Classical reference (mp proxy)",
    "rl_residual": "RL reference + PPO residual",
}
COLORS = {
    "mp_reference_proxy": "#4c566a",
    "rl_residual": "#2a9d8f",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--train-root",
        type=Path,
        default=Path("/home/ktj/omx_train_ws"),
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/arm_grasp_randomized_ppo.yaml"),
    )
    parser.add_argument(
        "--policy",
        type=Path,
        default=Path(
            "policies/latest/arm_delivery_residual_v2/arm_grasp_latest.zip"
        ),
    )
    parser.add_argument("--stage", default="sim2real_robust")
    parser.add_argument("--trials", type=int, default=50)
    parser.add_argument("--seed", type=int, default=20260717)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def resolve_under(root: Path, path: Path) -> Path:
    return path.expanduser().resolve() if path.is_absolute() else (root / path).resolve()


def run_phase(
    env: Any,
    model: PPO,
    controller: str,
    task: str,
    seed: int,
) -> dict[str, Any]:
    observation, initial_info = env.reset(seed=seed, options={"task": task})
    total_reward = 0.0
    inference_ns = 0
    final_info = initial_info
    terminated = False
    truncated = False
    step = -1

    for step in range(env.max_episode_steps):
        if controller == "rl_residual":
            started = time.perf_counter_ns()
            action, _ = model.predict(observation, deterministic=True)
            inference_ns += time.perf_counter_ns() - started
        else:
            action = np.zeros(env.action_space.shape, dtype=np.float32)

        observation, reward, terminated, truncated, final_info = env.step(action)
        total_reward += float(reward)
        if terminated or truncated:
            break

    steps = step + 1
    inference_calls = steps if controller == "rl_residual" else 0
    return {
        "task": task,
        "seed": seed,
        "bucket": str(initial_info["object_sample_bucket"]),
        "success": bool(final_info["is_success"]),
        "collision": bool(final_info.get("collision_failure", False)),
        "truncated": bool(truncated and not terminated),
        "steps": steps,
        "sim_time_s": steps * float(env.model.opt.timestep * env.frame_skip),
        "reward": total_reward,
        "reach_distance_m": float(final_info["reach_distance"]),
        "stay_error_rad": float(final_info["stay_error"]),
        "placement_error_m": float(final_info["placement_error"]),
        "inference_calls": inference_calls,
        "inference_total_ms": inference_ns / 1.0e6,
        "inference_mean_ms": (
            inference_ns / inference_calls / 1.0e6 if inference_calls else 0.0
        ),
    }


def flatten_trial(
    controller: str,
    trial: int,
    seed: int,
    pick: dict[str, Any],
    place: dict[str, Any],
) -> dict[str, Any]:
    cycle_success = bool(pick["success"] and place["success"])
    cycle_collision = bool(pick["collision"] or place["collision"])
    return {
        "controller": controller,
        "trial": trial,
        "seed": seed,
        "bucket": pick["bucket"],
        "pick_success": pick["success"],
        "pick_collision": pick["collision"],
        "pick_truncated": pick["truncated"],
        "pick_steps": pick["steps"],
        "pick_time_s": pick["sim_time_s"],
        "pick_reward": pick["reward"],
        "pick_reach_distance_m": pick["reach_distance_m"],
        "pick_stay_error_rad": pick["stay_error_rad"],
        "place_success": place["success"],
        "place_collision": place["collision"],
        "place_truncated": place["truncated"],
        "place_steps": place["steps"],
        "place_time_s": place["sim_time_s"],
        "place_reward": place["reward"],
        "place_placement_error_m": place["placement_error_m"],
        "place_stay_error_rad": place["stay_error_rad"],
        "cycle_success": cycle_success,
        "cycle_collision": cycle_collision,
        "cycle_steps": int(pick["steps"] + place["steps"]),
        "cycle_time_s": float(pick["sim_time_s"] + place["sim_time_s"]),
        "total_reward": float(pick["reward"] + place["reward"]),
        "inference_calls": int(
            pick["inference_calls"] + place["inference_calls"]
        ),
        "inference_total_ms": float(
            pick["inference_total_ms"] + place["inference_total_ms"]
        ),
        "inference_mean_ms": float(
            (
                pick["inference_total_ms"] + place["inference_total_ms"]
            )
            / max(1, pick["inference_calls"] + place["inference_calls"])
        ),
    }


def percentile(values: list[float], q: float) -> float | None:
    return float(np.percentile(values, q)) if values else None


def controller_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    successful_times = [
        float(row["cycle_time_s"]) for row in rows if row["cycle_success"]
    ]
    all_times = [float(row["cycle_time_s"]) for row in rows]
    pick_times = [
        float(row["pick_time_s"]) for row in rows if row["pick_success"]
    ]
    place_times = [
        float(row["place_time_s"]) for row in rows if row["place_success"]
    ]
    inference_means = [
        float(row["inference_mean_ms"])
        for row in rows
        if int(row["inference_calls"]) > 0
    ]
    count = len(rows)
    return {
        "trials": count,
        "cycle_successes": sum(bool(row["cycle_success"]) for row in rows),
        "cycle_success_rate": sum(bool(row["cycle_success"]) for row in rows)
        / count,
        "pick_success_rate": sum(bool(row["pick_success"]) for row in rows)
        / count,
        "place_success_rate": sum(bool(row["place_success"]) for row in rows)
        / count,
        "cycle_collision_rate": sum(bool(row["cycle_collision"]) for row in rows)
        / count,
        "mean_all_trial_time_s": float(np.mean(all_times)),
        "mean_successful_cycle_time_s": (
            float(np.mean(successful_times)) if successful_times else None
        ),
        "median_successful_cycle_time_s": percentile(successful_times, 50),
        "p95_successful_cycle_time_s": percentile(successful_times, 95),
        "mean_successful_pick_time_s": (
            float(np.mean(pick_times)) if pick_times else None
        ),
        "mean_successful_place_time_s": (
            float(np.mean(place_times)) if place_times else None
        ),
        "mean_inference_ms": (
            float(np.mean(inference_means)) if inference_means else 0.0
        ),
    }


def paired_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    by_controller = {
        controller: {int(row["trial"]): row for row in rows if row["controller"] == controller}
        for controller in CONTROLLERS
    }
    common_successes: list[tuple[dict[str, Any], dict[str, Any]]] = []
    for trial in sorted(by_controller[CONTROLLERS[0]]):
        classical = by_controller["mp_reference_proxy"][trial]
        rl = by_controller["rl_residual"][trial]
        if classical["cycle_success"] and rl["cycle_success"]:
            common_successes.append((classical, rl))

    deltas = [
        float(rl["cycle_time_s"]) - float(classical["cycle_time_s"])
        for classical, rl in common_successes
    ]
    classical_times = [
        float(classical["cycle_time_s"])
        for classical, _ in common_successes
    ]
    rl_times = [float(rl["cycle_time_s"]) for _, rl in common_successes]
    delta_ci = (None, None)
    if len(deltas) > 1:
        delta_ci = stats.t.interval(
            0.95,
            len(deltas) - 1,
            loc=float(np.mean(deltas)),
            scale=float(stats.sem(deltas)),
        )
    classical_mean = float(np.mean(classical_times)) if classical_times else None
    rl_mean = float(np.mean(rl_times)) if rl_times else None
    return {
        "common_successful_trials": len(common_successes),
        "classical_mean_time_s": classical_mean,
        "rl_mean_time_s": rl_mean,
        "mean_rl_minus_classical_time_s": (
            float(np.mean(deltas)) if deltas else None
        ),
        "median_rl_minus_classical_time_s": percentile(deltas, 50),
        "mean_delta_95ci_low_s": (
            float(delta_ci[0]) if delta_ci[0] is not None else None
        ),
        "mean_delta_95ci_high_s": (
            float(delta_ci[1]) if delta_ci[1] is not None else None
        ),
        "rl_time_reduction_percent": (
            -float(np.mean(deltas)) / classical_mean * 100.0
            if deltas and classical_mean
            else None
        ),
        "classical_to_rl_speed_ratio": (
            classical_mean / rl_mean if classical_mean and rl_mean else None
        ),
        "rl_faster_count": sum(delta < 0.0 for delta in deltas),
        "classical_faster_count": sum(delta > 0.0 for delta in deltas),
        "ties": sum(abs(delta) <= 1.0e-12 for delta in deltas),
        "two_sided_sign_test_p": (
            2.0 * (0.5 ** len(deltas))
            if deltas and all(delta < 0.0 for delta in deltas)
            else None
        ),
    }


def write_csv(rows: list[dict[str, Any]], output: Path) -> None:
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_trial_times(rows: list[dict[str, Any]], output: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 5.5), constrained_layout=True)
    for controller in CONTROLLERS:
        selected = [row for row in rows if row["controller"] == controller]
        trials = [int(row["trial"]) for row in selected]
        times = [float(row["cycle_time_s"]) for row in selected]
        ax.plot(
            trials,
            times,
            linewidth=1.4,
            color=COLORS[controller],
            alpha=0.82,
            label=LABELS[controller],
        )
        success_trials = [
            int(row["trial"]) for row in selected if row["cycle_success"]
        ]
        success_times = [
            float(row["cycle_time_s"])
            for row in selected
            if row["cycle_success"]
        ]
        failed_trials = [
            int(row["trial"]) for row in selected if not row["cycle_success"]
        ]
        failed_times = [
            float(row["cycle_time_s"])
            for row in selected
            if not row["cycle_success"]
        ]
        ax.scatter(
            success_trials,
            success_times,
            color=COLORS[controller],
            s=22,
            zorder=3,
        )
        ax.scatter(
            failed_trials,
            failed_times,
            color=COLORS[controller],
            marker="x",
            s=38,
            linewidth=1.5,
            zorder=4,
        )
    ax.set_title("Matched-seed Pick + Place cycle duration (50 trials)")
    ax.set_xlabel("Trial")
    ax.set_ylabel("Simulation time (s)")
    ax.set_xlim(0.5, max(int(row["trial"]) for row in rows) + 0.5)
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(loc="upper right")
    ax.text(
        0.01,
        0.02,
        "Circle: cycle success   X: pick or place failure",
        transform=ax.transAxes,
        fontsize=9,
        color="#444444",
    )
    fig.savefig(output, dpi=200)
    plt.close(fig)


def plot_summary(
    rows: list[dict[str, Any]],
    summaries: dict[str, dict[str, Any]],
    output: Path,
) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.8), constrained_layout=True)
    x = np.arange(len(CONTROLLERS))
    labels = ["Classical\nreference", "RL residual"]

    metrics = ("cycle_success_rate", "pick_success_rate", "place_success_rate")
    metric_labels = ("Cycle", "Pick", "Place")
    width = 0.23
    for index, (metric, metric_label) in enumerate(zip(metrics, metric_labels)):
        values = [summaries[controller][metric] * 100.0 for controller in CONTROLLERS]
        axes[0].bar(
            x + (index - 1) * width,
            values,
            width,
            label=metric_label,
            color=("#457b9d", "#2a9d8f", "#e9c46a")[index],
        )
        for position, value in zip(x + (index - 1) * width, values):
            axes[0].text(position, value + 1.2, f"{value:.0f}%", ha="center", fontsize=8)
    axes[0].set_title("Success rate")
    axes[0].set_xticks(x, labels)
    axes[0].set_ylim(0, 112)
    axes[0].set_ylabel("Rate (%)")
    axes[0].legend(loc="lower right", fontsize=8)
    axes[0].grid(True, axis="y", alpha=0.2)

    collision_values = [
        summaries[controller]["cycle_collision_rate"] * 100.0
        for controller in CONTROLLERS
    ]
    bars = axes[1].bar(x, collision_values, width=0.55, color=[COLORS[c] for c in CONTROLLERS])
    axes[1].bar_label(bars, fmt="%.0f%%", padding=3)
    axes[1].set_title("Cycle collision rate")
    axes[1].set_xticks(x, labels)
    axes[1].set_ylim(0, max(10.0, max(collision_values) * 1.25))
    axes[1].set_ylabel("Rate (%)")
    axes[1].grid(True, axis="y", alpha=0.2)

    successful_times = [
        [
            float(row["cycle_time_s"])
            for row in rows
            if row["controller"] == controller and row["cycle_success"]
        ]
        for controller in CONTROLLERS
    ]
    box = axes[2].boxplot(successful_times, tick_labels=labels, patch_artist=True)
    for patch, controller in zip(box["boxes"], CONTROLLERS):
        patch.set_facecolor(COLORS[controller])
        patch.set_alpha(0.75)
    axes[2].set_title("Successful cycle time")
    axes[2].set_ylabel("Simulation time (s)")
    axes[2].grid(True, axis="y", alpha=0.2)

    fig.suptitle("Classical reference proxy vs residual PPO, sim2real_robust")
    fig.savefig(output, dpi=200)
    plt.close(fig)


def markdown_value(value: Any, digits: int = 3) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def write_markdown(
    output: Path,
    args: argparse.Namespace,
    policy_path: Path,
    summaries: dict[str, dict[str, Any]],
    paired: dict[str, Any],
) -> None:
    sign_test_p = paired["two_sided_sign_test_p"]
    sign_test_text = f"{sign_test_p:.3e}" if sign_test_p is not None else "N/A"
    lines = [
        "# Classical reference proxy vs RL: 50-trial result",
        "",
        "| Item | Value |",
        "|---|---|",
        f"| Stage | `{args.stage}` |",
        f"| Trials | `{args.trials}` matched seeds |",
        "| Trial definition | Pick episode + Place episode |",
        f"| Master seed | `{args.seed}` |",
        f"| Policy | `{policy_path}` |",
        "| Control timestep | `0.02 s` |",
        "",
        "> The classical group is the environment's deterministic reference controller with the PPO residual fixed to zero. It is an mp_control proxy, not a replay of the C++ mp_control ROS node.",
        "",
        "## Summary",
        "",
        "| Metric | Classical reference proxy | RL residual |",
        "|---|---:|---:|",
    ]
    classical = summaries["mp_reference_proxy"]
    rl = summaries["rl_residual"]
    table_rows = (
        ("Cycle success", "cycle_successes", "count"),
        ("Cycle success rate", "cycle_success_rate", "percent"),
        ("Pick success rate", "pick_success_rate", "percent"),
        ("Place success rate", "place_success_rate", "percent"),
        ("Cycle collision rate", "cycle_collision_rate", "percent"),
        ("Mean successful cycle time (s)", "mean_successful_cycle_time_s", "float"),
        ("Median successful cycle time (s)", "median_successful_cycle_time_s", "float"),
        ("P95 successful cycle time (s)", "p95_successful_cycle_time_s", "float"),
        ("Mean successful Pick time (s)", "mean_successful_pick_time_s", "float"),
        ("Mean successful Place time (s)", "mean_successful_place_time_s", "float"),
        ("Mean policy inference (ms)", "mean_inference_ms", "float"),
    )
    for label, key, kind in table_rows:
        left = classical[key]
        right = rl[key]
        if kind == "percent":
            left_text = f"{left * 100.0:.1f}%"
            right_text = f"{right * 100.0:.1f}%"
        elif kind == "count":
            left_text = f"{left}/{args.trials}"
            right_text = f"{right}/{args.trials}"
        else:
            left_text = markdown_value(left)
            right_text = markdown_value(right)
        lines.append(f"| {label} | {left_text} | {right_text} |")

    lines.extend(
        [
            "",
            "## Paired successful trials",
            "",
            "| Metric | Result |",
            "|---|---:|",
            f"| Both controllers succeeded | {paired['common_successful_trials']} |",
            "| Classical paired mean time (s) | "
            f"{markdown_value(paired['classical_mean_time_s'])} |",
            "| RL paired mean time (s) | "
            f"{markdown_value(paired['rl_mean_time_s'])} |",
            "| Mean RL - classical time (s) | "
            f"{markdown_value(paired['mean_rl_minus_classical_time_s'])} |",
            "| Median RL - classical time (s) | "
            f"{markdown_value(paired['median_rl_minus_classical_time_s'])} |",
            "| Mean delta 95% CI (s) | "
            f"[{markdown_value(paired['mean_delta_95ci_low_s'])}, "
            f"{markdown_value(paired['mean_delta_95ci_high_s'])}] |",
            "| RL time reduction | "
            f"{markdown_value(paired['rl_time_reduction_percent'])}% |",
            "| Classical / RL speed ratio | "
            f"{markdown_value(paired['classical_to_rl_speed_ratio'])}x |",
            f"| RL faster | {paired['rl_faster_count']} |",
            f"| Classical faster | {paired['classical_faster_count']} |",
            f"| Ties | {paired['ties']} |",
            f"| Two-sided sign-test p | {sign_test_text} |",
            "",
            "## Figures",
            "",
            "![Trial cycle time](./trial_cycle_time.png)",
            "",
            "![Summary metrics](./summary_metrics.png)",
            "",
            "Raw per-trial values are stored in `trials.csv`; machine-readable aggregate values are stored in `summary.yaml`.",
        ]
    )
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    if args.trials < 1:
        raise ValueError("trials must be positive")
    train_root = args.train_root.expanduser().resolve()
    config_path = resolve_under(train_root, args.config)
    policy_path = resolve_under(train_root, args.policy)
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    torch.set_num_threads(1)
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass

    if str(train_root) not in sys.path:
        sys.path.insert(0, str(train_root))
    from envs import load_config, make_grasp_env

    config = load_config(config_path)
    model = PPO.load(policy_path, device="cpu")
    rng = np.random.default_rng(args.seed)
    seeds = [int(value) for value in rng.integers(0, 2**31 - 1, size=args.trials)]
    rows: list[dict[str, Any]] = []

    for controller in CONTROLLERS:
        env = make_grasp_env(config, stage_name=args.stage)
        env.start_grasped_probability = 0.0
        env.start_released_probability = 0.0
        for index, seed in enumerate(seeds, start=1):
            pick = run_phase(env, model, controller, "pick", seed)
            place = run_phase(env, model, controller, "place", seed)
            rows.append(flatten_trial(controller, index, seed, pick, place))
            print(
                f"controller={controller} trial={index}/{args.trials} "
                f"success={rows[-1]['cycle_success']} "
                f"collision={rows[-1]['cycle_collision']} "
                f"time={rows[-1]['cycle_time_s']:.3f}s",
                flush=True,
            )
        env.close()

    summaries = {
        controller: controller_summary(
            [row for row in rows if row["controller"] == controller]
        )
        for controller in CONTROLLERS
    }
    paired = paired_summary(rows)
    report = {
        "experiment": {
            "stage": args.stage,
            "trials": args.trials,
            "trial_definition": "pick episode plus place episode",
            "master_seed": args.seed,
            "seeds": seeds,
            "policy": str(policy_path),
            "config": str(config_path),
            "classical_definition": "reference controller with zero PPO residual",
            "classical_limitation": "proxy only; not C++ mp_control ROS replay",
            "control_timestep_s": 0.02,
        },
        "controllers": summaries,
        "paired_successful_trials": paired,
    }

    write_csv(rows, output_dir / "trials.csv")
    (output_dir / "summary.yaml").write_text(
        yaml.safe_dump(report, sort_keys=False), encoding="utf-8"
    )
    plot_trial_times(rows, output_dir / "trial_cycle_time.png")
    plot_summary(rows, summaries, output_dir / "summary_metrics.png")
    write_markdown(
        output_dir / "README.md", args, policy_path, summaries, paired
    )
    print(yaml.safe_dump(report, sort_keys=False))


if __name__ == "__main__":
    main()
