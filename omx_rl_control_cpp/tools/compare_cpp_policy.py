#!/usr/bin/env python3
"""Compare the C++ actor output with Stable-Baselines3 deterministic predict."""

import argparse
from pathlib import Path
import subprocess

import numpy as np


def cpp_predict(probe: Path, actor: Path, observation: np.ndarray):
    """Run the dependency-free C++ policy probe for one observation."""
    command = [str(probe), str(actor), *[f'{value:.9g}' for value in observation]]
    result = subprocess.run(
        command, check=True, capture_output=True, text=True)
    return np.fromstring(result.stdout, sep=' ', dtype=np.float64)


def main():
    """Check zero, edge, and deterministic random observations."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--actor', type=Path, required=True)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=128)
    parser.add_argument('--tolerance', type=float, default=2.0e-6)
    arguments = parser.parse_args()

    from stable_baselines3 import PPO

    model = PPO.load(str(arguments.policy), device='cpu')
    rng = np.random.default_rng(20260802)
    observations = [
        np.zeros(33, dtype=np.float32),
        np.ones(33, dtype=np.float32),
        -np.ones(33, dtype=np.float32),
    ]
    observations.extend(
        rng.uniform(-1.0, 1.0, size=33).astype(np.float32)
        for _ in range(arguments.samples)
    )

    maximum_error = 0.0
    for index, observation in enumerate(observations):
        python_action, _ = model.predict(observation, deterministic=True)
        cpp_action = cpp_predict(arguments.probe, arguments.actor, observation)
        if cpp_action.shape != (4,):
            raise RuntimeError(f'C++ sample {index} returned {cpp_action.shape}')
        error = float(np.max(np.abs(cpp_action - python_action)))
        maximum_error = max(maximum_error, error)
        if error > arguments.tolerance:
            raise RuntimeError(
                f'parity failure at sample {index}: max_error={error:.9g}')

    print(
        f'PASS samples={len(observations)} max_abs_error={maximum_error:.9g} '
        f'tolerance={arguments.tolerance:.9g}')


if __name__ == '__main__':
    main()
