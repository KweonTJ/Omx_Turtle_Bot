#!/usr/bin/env python3
"""Offline parity probe only. No ROS, serial access, or motion command code."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model-dir', type=Path, required=True)
    p.add_argument('--backend', choices=['torchscript', 'onnx'], default='torchscript')
    p.add_argument('--device', choices=['cpu', 'cuda'], default='cpu')
    args = p.parse_args()
    root = args.model_dir.resolve()
    for line in (root/'SHA256SUMS').read_text().splitlines():
        expected, name = line.split('  ', 1)
        target = (root/name).resolve()
        if root not in target.parents or hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise ValueError('Checksum mismatch or invalid path: '+name)
    fixture = np.load(root/'parity_vectors.npz', allow_pickle=False)
    obs, expected = fixture['observations'], fixture['actions']
    if obs.ndim != 2 or obs.shape[1] != 69 or expected.shape != (len(obs), 4):
        raise ValueError('Expected v6 69D observation / 4D action')
    if not np.isfinite(obs).all() or not np.isfinite(expected).all():
        raise ValueError('Non-finite fixture')
    if args.backend == 'torchscript':
        import torch
        torch.set_num_threads(1)
        if args.device == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA requested but unavailable; no silent fallback')
        model = torch.jit.load(str(root/'actor.ts'), map_location=args.device).eval()
        batch = torch.from_numpy(obs).to(args.device)
        one = batch[:1]
        sync = torch.cuda.synchronize if args.device == 'cuda' else lambda: None
        with torch.inference_mode():
            actual = model(batch).cpu().numpy()
            for _ in range(20):
                model(one)
            sync()
            start = time.perf_counter()
            for _ in range(500):
                model(one)
                sync()
            us = (time.perf_counter()-start)*1e6/500
        version = torch.__version__
    else:
        import onnxruntime as ort
        provider = 'CUDAExecutionProvider' if args.device == 'cuda' else 'CPUExecutionProvider'
        if provider not in ort.get_available_providers():
            raise RuntimeError('Requested ONNX provider unavailable: '+provider)
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = opts.inter_op_num_threads = 1
        model = ort.InferenceSession(str(root/'actor.onnx'), opts, providers=[provider])
        if model.get_providers()[0] != provider:
            raise RuntimeError('Requested ONNX provider not active')
        actual = model.run(['action'], {'observation': obs})[0]
        for _ in range(20):
            model.run(None, {'observation': obs[:1]})
        start = time.perf_counter()
        for _ in range(500):
            model.run(None, {'observation': obs[:1]})
        us = (time.perf_counter()-start)*1e6/500
        version = ort.__version__
    np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-5)
    print(json.dumps(dict(backend=args.backend, device=args.device, version=version,
        samples=len(obs), max_abs_error=float(np.max(np.abs(actual-expected))),
        mean_inference_us=us, motor_commands_sent=0, hardware_motion_validated=False), indent=2))


if __name__ == '__main__':
    main()
