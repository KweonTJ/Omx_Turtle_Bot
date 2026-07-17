#!/usr/bin/env python3
# flake8: noqa
"""Run independent GPU Gazebo server trials and aggregate their CSV rows."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', required=True)
    parser.add_argument('--residual-override', required=True, type=float)
    parser.add_argument('--trials', type=int, default=50)
    parser.add_argument('--seed', type=int, default=20260717)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--world', required=True, type=Path)
    parser.add_argument('--ros-domain-id', type=int, default=190)
    parser.add_argument('--trial-timeout', type=float, default=20.0)
    parser.add_argument('--ready-timeout', type=float, default=30.0)
    parser.add_argument('--log-dir', type=Path, default=Path('/tmp/omx_gazebo_batch'))
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--startup-retries', type=int, default=2)
    return parser.parse_args()


def stop_process_group(process: subprocess.Popen) -> None:
    def signal_group(sig: signal.Signals) -> None:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    signal_group(signal.SIGINT)
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        pass
    # ign gazebo can outlive the ros2 launch parent while retaining its PGID.
    signal_group(signal.SIGTERM)
    time.sleep(1.0)
    signal_group(signal.SIGKILL)
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        pass


def write_rows(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    if args.trials < 1:
        raise ValueError('trials must be positive')
    if args.startup_retries < 0:
        raise ValueError('startup-retries must be non-negative')
    args.log_dir.mkdir(parents=True, exist_ok=True)
    runner = Path(__file__).with_name('gazebo_cycle_runner.py')
    rows: list[dict[str, str]] = []
    if args.resume and args.output.exists():
        with args.output.open(encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
        if len(rows) > args.trials:
            raise ValueError('existing output has more rows than trials')
        print(f'resuming at trial {len(rows) + 1}/{args.trials}', flush=True)

    with tempfile.TemporaryDirectory(prefix='omx_gazebo_trials_') as temp_dir:
        temp_path = Path(temp_dir)
        for index in range(len(rows), args.trials):
            trial = index + 1
            environment = os.environ.copy()
            launch_command = [
                'ros2', 'launch', 'omx_rl_control', 'rl_gazebo.launch.py',
                'start_rviz:=false',
                f'residual_action_scale_override:={args.residual_override}',
                'gz_args:=-r -s --headless-rendering '
                f'--render-engine-server ogre2 {args.world}',
            ]
            trial_csv = temp_path / f'trial_{trial:03d}.csv'
            runner_command = [
                sys.executable, str(runner),
                '--controller', args.controller,
                '--trials', '1',
                '--seed', str(args.seed),
                '--sample-offset', str(index),
                '--output', str(trial_csv),
                '--trial-timeout', str(args.trial_timeout),
                '--ready-timeout', str(args.ready_timeout),
            ]
            result = None
            launch_log_path = None
            for attempt in range(args.startup_retries + 1):
                domain_id = 1 + (
                    args.ros_domain_id - 1
                    + index * (args.startup_retries + 1)
                    + attempt
                ) % 231
                environment.update({
                    'ROS_DOMAIN_ID': str(domain_id),
                    'ROS_LOG_DIR': str(
                        args.log_dir
                        / f'ros_trial_{trial:03d}_attempt_{attempt + 1}'
                    ),
                    'IGN_PARTITION': (
                        f'omx_{args.controller}_{trial:03d}_{attempt + 1}'
                    ),
                    '__NV_PRIME_RENDER_OFFLOAD': '1',
                    '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
                })
                launch_log_path = args.log_dir / (
                    f'trial_{trial:03d}_attempt_{attempt + 1}.log')
                trial_csv.unlink(missing_ok=True)
                with launch_log_path.open('w', encoding='utf-8') as launch_log:
                    server = subprocess.Popen(
                        launch_command,
                        env=environment,
                        stdout=launch_log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                        text=True,
                    )
                    try:
                        result = subprocess.run(
                            runner_command,
                            env=environment,
                            capture_output=True,
                            text=True,
                            timeout=(
                                args.ready_timeout
                                + 2.0 * args.trial_timeout
                                + 20.0
                            ),
                            check=False,
                        )
                    finally:
                        stop_process_group(server)
                if result.returncode == 0 and trial_csv.exists():
                    break
                if attempt < args.startup_retries:
                    print(
                        f'trial {trial} startup attempt {attempt + 1} '
                        'failed; retrying',
                        flush=True,
                    )
                    time.sleep(2.0)

            if result is None or result.returncode != 0 or not trial_csv.exists():
                return_code = result.returncode if result is not None else 'none'
                detail = '' if result is None else (
                    result.stderr.strip() or result.stdout.strip())
                raise RuntimeError(
                    f'trial {trial} runner failed with {return_code}: '
                    f'{detail} '
                    f'(launch log: {launch_log_path})'
                )
            with trial_csv.open(encoding='utf-8', newline='') as stream:
                row = next(csv.DictReader(stream))
            rows.append(row)
            write_rows(args.output, rows)
            print(result.stdout.strip(), flush=True)
            time.sleep(1.0)


if __name__ == '__main__':
    main()
