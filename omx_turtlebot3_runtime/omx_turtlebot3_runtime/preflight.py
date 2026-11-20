"""Read actual OpenCR state and model readiness. This CLI has no execute option."""
import argparse
import json
from pathlib import Path
import time
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
from .core import commissioning_issues
from .node import validate_config, json_value
from .opencr import OpenCR


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(get_package_share_directory('omx_turtlebot3_runtime'))
    parser.add_argument('--config', type=Path, default=root/'config/runtime.yaml')
    parser.add_argument('--seconds', type=float, default=3.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not .2 <= args.seconds <= 30:
        raise ValueError('Read-only capture duration must be .2..30 seconds')
    cfg = yaml.safe_load(args.config.read_text())
    validate_config(cfg)
    samples = []
    with args.output.open('x') as output:
        transport = OpenCR(cfg, execute=False)
        try:
            start = time.monotonic()
            while time.monotonic()-start < args.seconds:
                time.sleep(.05)
                samples.append(vars(transport.read()))
        finally:
            transport.close()
        report = dict(samples=samples, motor_writes=transport.writes,
                      commissioning_issues=commissioning_issues(cfg),
                      current_raw_range=[min(s['gripper_current_raw'] for s in samples), max(s['gripper_current_raw'] for s in samples)],
                      note='Idle capture is not a calibrated grip/load threshold. No motor command sent.')
        json.dump(report, output, default=json_value, indent=2, allow_nan=False)
        print(json.dumps({k: v for k, v in report.items() if k != 'samples'}))


if __name__ == '__main__':
    main()
