#!/usr/bin/env python3
"""Export the deterministic SB3 PPO actor to the small C++ runtime format."""

import argparse
import hashlib
from pathlib import Path
import struct

import numpy as np
import yaml


MAGIC = b'OMXACTR1'
FORMAT_VERSION = 1


def sha256(path: Path) -> str:
    """Return the SHA-256 digest for one file."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def tensor(state, name, shape):
    """Extract one finite float32 tensor with an exact shape."""
    value = state[name].detach().cpu().numpy().astype('<f4', copy=False)
    if value.shape != shape:
        raise ValueError(f'{name}: expected {shape}, got {value.shape}')
    if not np.isfinite(value).all():
        raise ValueError(f'{name}: contains NaN or Inf')
    return np.ascontiguousarray(value)


def main():
    """Load an SB3 archive and export only its deterministic actor layers."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--policy', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--model-version', required=True)
    arguments = parser.parse_args()

    from stable_baselines3 import PPO

    model = PPO.load(str(arguments.policy), device='cpu')
    state = model.policy.state_dict()
    arrays = [
        tensor(state, 'mlp_extractor.policy_net.0.weight', (256, 33)),
        tensor(state, 'mlp_extractor.policy_net.0.bias', (256,)),
        tensor(state, 'mlp_extractor.policy_net.2.weight', (256, 256)),
        tensor(state, 'mlp_extractor.policy_net.2.bias', (256,)),
        tensor(state, 'action_net.weight', (4, 256)),
        tensor(state, 'action_net.bias', (4,)),
    ]

    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with arguments.output.open('wb') as stream:
        stream.write(MAGIC)
        stream.write(struct.pack('<5I', FORMAT_VERSION, 33, 256, 256, 4))
        for array in arrays:
            stream.write(array.tobytes(order='C'))

    metadata = {
        'format': 'omx_sb3_actor',
        'format_version': FORMAT_VERSION,
        'model_version': arguments.model_version,
        'source_policy': arguments.policy.name,
        'source_policy_sha256': sha256(arguments.policy),
        'actor_file': arguments.output.name,
        'actor_sha256': sha256(arguments.output),
        'observation_size': 33,
        'hidden_layers': [256, 256],
        'hidden_activation': 'tanh',
        'action_size': 4,
        'action_postprocess': 'clip[-1,1]',
        'deterministic': True,
    }
    arguments.metadata.parent.mkdir(parents=True, exist_ok=True)
    arguments.metadata.write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')

    print(f'actor={arguments.output}')
    print(f'actor_sha256={metadata["actor_sha256"]}')
    print(f'source_policy_sha256={metadata["source_policy_sha256"]}')


if __name__ == '__main__':
    main()
