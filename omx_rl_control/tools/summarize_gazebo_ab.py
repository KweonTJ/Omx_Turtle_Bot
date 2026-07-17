#!/usr/bin/env python3
# flake8: noqa
"""Summarize independent GPU Gazebo reference and residual PPO trials."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml


CONTROLLERS = {
    'reference': 'Reference only',
    'rl': 'Reference + PPO residual',
}
COLORS = {
    'reference': '#3B6EA8',
    'rl': '#D55E00',
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', required=True, type=Path)
    return parser.parse_args()


def boolean(series: pd.Series) -> pd.Series:
    return series.astype(str).str.lower().eq('true')


def read_trials(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path)
    for column in ('state_success', 'physics_success', 'cycle_success'):
        frame[column] = boolean(frame[column])
    return frame


def percentile(values: pd.Series, quantile: float) -> float:
    return float(np.quantile(values.to_numpy(dtype=float), quantile))


def trial_metrics(frame: pd.DataFrame) -> dict:
    successes = frame[frame['cycle_success']]
    completed = frame[frame['state_success']]
    times = successes['cycle_time_s']
    failure_states = frame.loc[
        ~frame['state_success'], 'place_state'
    ].value_counts().to_dict()
    return {
        'trials': int(len(frame)),
        'state_success_count': int(frame['state_success'].sum()),
        'physics_success_count': int(frame['physics_success'].sum()),
        'cycle_success_count': int(frame['cycle_success'].sum()),
        'cycle_success_rate': float(frame['cycle_success'].mean()),
        'physical_failure_after_complete_count': int((
            frame['state_success'] & ~frame['physics_success']
        ).sum()),
        'failure_states': {
            str(key): int(value) for key, value in failure_states.items()
        },
        'successful_cycle_time_s': {
            'mean': float(times.mean()),
            'median': float(times.median()),
            'p95': percentile(times, 0.95),
            'min': float(times.min()),
            'max': float(times.max()),
        },
        'successful_pick_time_s_mean': float(
            successes['pick_time_s'].mean()),
        'successful_place_time_s_mean': float(
            successes['place_time_s'].mean()),
        'completed_final_xy_error_mm': {
            'mean': float(completed['final_xy_error_m'].mean() * 1000.0),
            'median': float(completed['final_xy_error_m'].median() * 1000.0),
            'p95': float(percentile(
                completed['final_xy_error_m'], 0.95) * 1000.0),
        },
    }


def read_gpu(path: Path) -> pd.DataFrame:
    columns = ['timestamp', 'utilization_pct', 'memory_mib', 'power_w', 'temp_c']
    frame = pd.read_csv(path, names=columns, skipinitialspace=True)
    frame['timestamp'] = pd.to_datetime(frame['timestamp'])
    return frame


def gpu_metrics(frame: pd.DataFrame) -> dict:
    active = frame[frame['memory_mib'] >= 300.0]
    return {
        'samples': int(len(frame)),
        'active_samples': int(len(active)),
        'active_definition': 'memory_mib >= 300',
        'overall_utilization_mean_pct': float(
            frame['utilization_pct'].mean()),
        'active_utilization_mean_pct': float(
            active['utilization_pct'].mean()),
        'active_utilization_p95_pct': percentile(
            active['utilization_pct'], 0.95),
        'utilization_max_pct': float(frame['utilization_pct'].max()),
        'memory_max_mib': float(frame['memory_mib'].max()),
        'active_power_mean_w': float(active['power_w'].mean()),
        'power_max_w': float(frame['power_w'].max()),
        'temperature_max_c': float(frame['temp_c'].max()),
    }


def paired_metrics(reference: pd.DataFrame, rl: pd.DataFrame) -> dict:
    merged = reference.merge(
        rl,
        on='trial',
        suffixes=('_reference', '_rl'),
        validate='one_to_one',
    )
    both = merged[
        merged['cycle_success_reference'] & merged['cycle_success_rl']
    ].copy()
    delta = both['cycle_time_s_rl'] - both['cycle_time_s_reference']
    return {
        'both_success_count': int(len(both)),
        'reference_only_success_count': int((
            merged['cycle_success_reference']
            & ~merged['cycle_success_rl']
        ).sum()),
        'rl_only_success_count': int((
            ~merged['cycle_success_reference']
            & merged['cycle_success_rl']
        ).sum()),
        'neither_success_count': int((
            ~merged['cycle_success_reference']
            & ~merged['cycle_success_rl']
        ).sum()),
        'rl_minus_reference_cycle_time_s': {
            'mean': float(delta.mean()),
            'median': float(delta.median()),
            'p95': percentile(delta, 0.95),
        },
        'rl_faster_count_on_both_success': int((delta < 0.0).sum()),
        'reference_faster_count_on_both_success': int((delta > 0.0).sum()),
        'tie_count_on_both_success': int((delta == 0.0).sum()),
    }


def verify_inputs(reference: pd.DataFrame, rl: pd.DataFrame) -> None:
    if len(reference) != 50 or len(rl) != 50:
        raise ValueError('both controllers must contain exactly 50 trials')
    expected = np.arange(1, 51)
    if not np.array_equal(reference['trial'].to_numpy(), expected):
        raise ValueError('reference trial indices are not 1..50')
    if not np.array_equal(rl['trial'].to_numpy(), expected):
        raise ValueError('RL trial indices are not 1..50')
    for column in ('seed', 'object_x_m', 'object_y_m', 'object_yaw_rad'):
        if not np.allclose(
            reference[column].to_numpy(dtype=float),
            rl[column].to_numpy(dtype=float),
            rtol=0.0,
            atol=1.0e-12,
        ):
            raise ValueError(f'matched trial mismatch: {column}')


def plot_trial_times(path: Path, frames: dict[str, pd.DataFrame]) -> None:
    figure, axis = plt.subplots(figsize=(12, 5.2))
    for key, frame in frames.items():
        success = frame['cycle_success']
        axis.plot(
            frame.loc[success, 'trial'],
            frame.loc[success, 'cycle_time_s'],
            marker='o', linestyle='none', markersize=5,
            color=COLORS[key], label=f'{CONTROLLERS[key]} success',
        )
        axis.plot(
            frame.loc[~success, 'trial'],
            frame.loc[~success, 'cycle_time_s'],
            marker='x', linestyle='none', markersize=6,
            color=COLORS[key], alpha=0.55,
            label=f'{CONTROLLERS[key]} failure',
        )
    axis.set_xlabel('Matched trial')
    axis.set_ylabel('Cycle wall time (s)')
    axis.set_title('Independent GPU Gazebo server trials')
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=2)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def plot_summary(path: Path, summary: dict) -> None:
    keys = list(CONTROLLERS)
    labels = [CONTROLLERS[key] for key in keys]
    rates = [summary['controllers'][key]['cycle_success_rate'] * 100 for key in keys]
    means = [summary['controllers'][key]['successful_cycle_time_s']['mean'] for key in keys]
    p95s = [summary['controllers'][key]['successful_cycle_time_s']['p95'] for key in keys]
    errors = [summary['controllers'][key]['completed_final_xy_error_mm']['median'] for key in keys]
    colors = [COLORS[key] for key in keys]
    figure, axes = plt.subplots(1, 3, figsize=(13, 4.5))
    axes[0].bar(labels, rates, color=colors)
    axes[0].set_ylabel('Cycle success (%)')
    axes[0].set_ylim(0, 100)
    axes[1].bar(labels, means, color=colors, label='Mean')
    axes[1].scatter(labels, p95s, color='black', marker='_', s=240, label='p95')
    axes[1].set_ylabel('Successful cycle time (s)')
    axes[1].legend()
    axes[2].bar(labels, errors, color=colors)
    axes[2].set_ylabel('Median final XY error (mm)')
    for axis in axes:
        axis.tick_params(axis='x', rotation=12)
        axis.grid(True, axis='y', alpha=0.2)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def plot_gpu(path: Path, frames: dict[str, pd.DataFrame]) -> None:
    figure, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
    for axis, (key, frame) in zip(axes, frames.items()):
        elapsed = (frame['timestamp'] - frame['timestamp'].iloc[0]).dt.total_seconds()
        axis.plot(elapsed, frame['utilization_pct'], color=COLORS[key], linewidth=0.8)
        axis.set_ylabel('GPU util (%)')
        axis.set_title(CONTROLLERS[key])
        axis.set_ylim(0, 100)
        axis.grid(True, alpha=0.2)
        memory_axis = axis.twinx()
        memory_axis.plot(elapsed, frame['memory_mib'], color='#555555', alpha=0.3, linewidth=0.7)
        memory_axis.set_ylabel('Memory (MiB)')
    axes[-1].set_xlabel('Experiment elapsed time (s)')
    figure.suptitle('RTX 4050 telemetry during independent Gazebo server trials')
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def write_readme(path: Path, summary: dict) -> None:
    ref = summary['controllers']['reference']
    rl = summary['controllers']['rl']
    paired = summary['paired']
    ref_gpu = summary['gpu']['reference']
    rl_gpu = summary['gpu']['rl']
    lines = [
        '# GPU Gazebo server 50회 A/B 결과',
        '',
        '> Reference 군은 PPO residual을 0으로 고정한 대리군이며 실제 C++ `mp_control` 실행이 아니다.',
        '',
        '| 지표 | Reference only | PPO residual |',
        '|---|---:|---:|',
        f"| Cycle 성공 | {ref['cycle_success_count']}/50 ({ref['cycle_success_rate'] * 100:.1f}%) | {rl['cycle_success_count']}/50 ({rl['cycle_success_rate'] * 100:.1f}%) |",
        f"| 상태 완료 | {ref['state_success_count']}/50 | {rl['state_success_count']}/50 |",
        f"| 완료 후 물리 실패 | {ref['physical_failure_after_complete_count']} | {rl['physical_failure_after_complete_count']} |",
        f"| 성공 cycle 평균 | {ref['successful_cycle_time_s']['mean']:.3f} s | {rl['successful_cycle_time_s']['mean']:.3f} s |",
        f"| 성공 cycle 중앙값 | {ref['successful_cycle_time_s']['median']:.3f} s | {rl['successful_cycle_time_s']['median']:.3f} s |",
        f"| 성공 cycle p95 | {ref['successful_cycle_time_s']['p95']:.3f} s | {rl['successful_cycle_time_s']['p95']:.3f} s |",
        f"| 완료 XY 오차 중앙값 | {ref['completed_final_xy_error_mm']['median']:.1f} mm | {rl['completed_final_xy_error_mm']['median']:.1f} mm |",
        f"| GPU active 평균 | {ref_gpu['active_utilization_mean_pct']:.1f}% | {rl_gpu['active_utilization_mean_pct']:.1f}% |",
        f"| GPU 최대 | {ref_gpu['utilization_max_pct']:.0f}% | {rl_gpu['utilization_max_pct']:.0f}% |",
        f"| GPU 최대 메모리 | {ref_gpu['memory_max_mib']:.0f} MiB | {rl_gpu['memory_max_mib']:.0f} MiB |",
        '',
        '## Paired 결과',
        '',
        f"- 두 제어기 모두 성공: {paired['both_success_count']}/50",
        f"- Reference만 성공: {paired['reference_only_success_count']}/50",
        f"- RL만 성공: {paired['rl_only_success_count']}/50",
        f"- 둘 다 실패: {paired['neither_success_count']}/50",
        f"- 공통 성공에서 RL-reference 평균 시간 차이: {paired['rl_minus_reference_cycle_time_s']['mean']:+.3f} s",
        f"- 공통 성공에서 RL이 빠른 회차: {paired['rl_faster_count_on_both_success']}/{paired['both_success_count']}",
        '',
        '## 조건',
        '',
        '- 회차마다 Gazebo server, ros2_control, PPO node를 새로 시작했다.',
        '- Gazebo server는 `-s --headless-rendering`, OGRE2 Sensors, NVIDIA PRIME offload를 사용했다.',
        '- PPO 추론은 RTX 4050 `cuda:0`에서 수행했다.',
        '- ODE 물리와 ROS 2 executor는 Gazebo 구조상 CPU에서 수행된다.',
        '- 물리 성공 허용치는 타워 중심 XY 25 mm, 상자 중심 Z 12 mm다.',
    ]
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> None:
    args = parse_args()
    results = args.results_dir
    reference = read_trials(results / 'reference_only.csv')
    rl = read_trials(results / 'rl_residual.csv')
    verify_inputs(reference, rl)
    gpu_frames = {
        'reference': read_gpu(results / 'reference_gpu.csv'),
        'rl': read_gpu(results / 'rl_gpu.csv'),
    }
    summary = {
        'experiment': {
            'master_seed': 20260717,
            'trials_per_controller': 50,
            'independent_gazebo_server_per_trial': True,
            'gpu': 'NVIDIA GeForce RTX 4050 Laptop GPU',
            'physics': 'Gazebo ODE on CPU',
            'rendering': 'Gazebo server OGRE2 headless on NVIDIA GPU',
            'policy_device': 'cuda:0',
            'final_xy_tolerance_m': 0.025,
            'final_z_tolerance_m': 0.012,
        },
        'controllers': {
            'reference': trial_metrics(reference),
            'rl': trial_metrics(rl),
        },
        'paired': paired_metrics(reference, rl),
        'gpu': {
            key: gpu_metrics(frame) for key, frame in gpu_frames.items()
        },
    }
    with (results / 'summary.yaml').open('w', encoding='utf-8') as stream:
        yaml.safe_dump(summary, stream, sort_keys=False, allow_unicode=True)
    combined = pd.concat([
        reference.assign(group='reference'),
        rl.assign(group='rl'),
    ], ignore_index=True)
    combined.to_csv(results / 'combined_trials.csv', index=False)
    plot_trial_times(
        results / 'gazebo_trial_cycle_time.png',
        {'reference': reference, 'rl': rl},
    )
    plot_summary(results / 'gazebo_summary_metrics.png', summary)
    plot_gpu(results / 'gpu_telemetry.png', gpu_frames)
    write_readme(results / 'README.md', summary)
    print(yaml.safe_dump(summary, sort_keys=False, allow_unicode=True))


if __name__ == '__main__':
    main()
