"""Validation helpers for a deployed Stable-Baselines3 policy."""

from dataclasses import dataclass
import hashlib
from pathlib import Path
from typing import Any

import yaml


class ContractError(RuntimeError):
    """Raised when a deployed policy violates its runtime contract."""


@dataclass(frozen=True)
class PolicyContract:
    """Validated policy metadata required by the ROS runtime."""

    root: Path
    policy_path: Path
    version: str
    observation_size: int
    action_size: int
    joint_names: tuple[str, ...]
    control_period_s: float
    action_scale: tuple[float, ...]
    action_filter_coefficient: float
    residual_action_scale: float
    stay_joint_positions: tuple[float, ...]
    ros_to_training_offset_xyz: tuple[float, ...]
    training_frame: str
    ros_target_frame: str


def sha256_file(path: Path) -> str:
    """Return a file SHA-256 without loading the whole artifact in memory."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ContractError(f'Missing policy artifact: {path}')
    content = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(content, dict):
        raise ContractError(f'Expected a YAML mapping: {path}')
    return content


def _require_equal(label: str, actual: Any, expected: Any) -> None:
    if actual != expected:
        raise ContractError(
            f'{label} mismatch: expected={expected!r}, actual={actual!r}')


def load_policy_contract(root: str | Path) -> PolicyContract:
    """Validate checksums and return the deployed policy contract."""
    artifact_root = Path(root).expanduser().resolve()
    manifest = _load_yaml(artifact_root / 'runtime_manifest.yaml')
    metadata = _load_yaml(artifact_root / 'policy_metadata.yaml')

    _require_equal('manifest_version', manifest.get('manifest_version'), 1)
    files = manifest.get('files')
    if not isinstance(files, dict) or not files:
        raise ContractError('runtime manifest files must be a non-empty mapping')
    for relative_name, expected_hash in files.items():
        artifact_path = artifact_root / str(relative_name)
        if not artifact_path.is_file():
            raise ContractError(f'Missing policy artifact: {artifact_path}')
        actual_hash = sha256_file(artifact_path)
        if actual_hash != str(expected_hash):
            raise ContractError(
                f'Checksum mismatch for {relative_name}: '
                f'expected={expected_hash}, actual={actual_hash}')

    contract = manifest.get('contract')
    if not isinstance(contract, dict):
        raise ContractError('runtime manifest contract must be a mapping')

    expected_joints = tuple(str(name) for name in contract['joint_names'])
    frame_offset = tuple(
        float(value)
        for value in contract['ros_to_training_offset_xyz']
    )
    if len(frame_offset) != 3:
        raise ContractError(
            'ros_to_training_offset_xyz must contain three values'
        )
    metadata_joints = tuple(str(name) for name in metadata['joint_names'])
    _require_equal('policy_version', metadata.get('policy_version'),
                   manifest.get('policy_version'))
    _require_equal('observation_schema_version',
                   metadata.get('observation_schema_version'), 1)
    _require_equal('action_schema_version',
                   metadata.get('action_schema_version'), 2)
    _require_equal('observation_size', metadata.get('observation_size'),
                   int(contract['observation_size']))
    _require_equal('action_size', metadata.get('action_size'),
                   int(contract['action_size']))
    _require_equal('joint_names', metadata_joints, expected_joints)
    _require_equal('control_mode', metadata.get('control_mode'),
                   'reference_plus_residual')
    _require_equal('manifest control_mode', contract.get('control_mode'),
                   metadata.get('control_mode'))

    training_config = artifact_root / 'training_config.yaml'
    config_hash = sha256_file(training_config)
    _require_equal('training config checksum', config_hash,
                   str(metadata.get('config_sha256')))

    policy_name = str(manifest.get('policy_file', 'policy.zip'))
    policy_path = artifact_root / policy_name
    if policy_name not in files:
        raise ContractError('policy_file must be covered by manifest checksums')

    return PolicyContract(
        root=artifact_root,
        policy_path=policy_path,
        version=str(metadata['policy_version']),
        observation_size=int(metadata['observation_size']),
        action_size=int(metadata['action_size']),
        joint_names=metadata_joints,
        control_period_s=float(metadata['control_period_s']),
        action_scale=tuple(float(value) for value in metadata['action_scale']),
        action_filter_coefficient=float(
            metadata['action_filter_coefficient']),
        residual_action_scale=float(metadata['residual_action_scale']),
        stay_joint_positions=tuple(
            float(value) for value in metadata['stay_joint_positions']),
        ros_to_training_offset_xyz=frame_offset,
        training_frame=str(contract['training_frame']),
        ros_target_frame=str(contract['ros_target_frame']),
    )


def validate_policy_spaces(model: Any, contract: PolicyContract) -> None:
    """Check the loaded SB3 model spaces against the manifest."""
    observation_shape = tuple(model.observation_space.shape or ())
    action_shape = tuple(model.action_space.shape or ())
    _require_equal('policy observation space', observation_shape,
                   (contract.observation_size,))
    _require_equal('policy action space', action_shape,
                   (contract.action_size,))
