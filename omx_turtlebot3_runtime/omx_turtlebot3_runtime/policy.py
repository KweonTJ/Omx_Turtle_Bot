"""Immutable model checksums, explicit backend/device, no fallback policy."""
import hashlib
import json
from pathlib import Path
import numpy as np
from .core import Fault, finite


class Actor:
    def __init__(self, root, backend='torchscript', device='cpu'):
        root = Path(root).resolve()
        for line in (root/'SHA256SUMS').read_text().splitlines():
            digest, name = line.split('  ', 1)
            path = (root/name).resolve()
            if root not in path.parents or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise Fault('Policy checksum mismatch: '+name)
        manifest = json.loads((root/'manifest.json').read_text())
        if (manifest['model_version'], manifest['observation_size'], manifest['action_size']) != ('omx_turtlebot3_v6', 69, 4):
            raise Fault('Wrong model observation/action contract')
        self.backend = backend
        if backend == 'torchscript':
            import torch
            if device not in ('cpu', 'cuda') or device == 'cuda' and not torch.cuda.is_available():
                raise Fault('Requested Torch device unavailable')
            torch.set_num_threads(1)
            self.torch, self.device = torch, device
            self.model = torch.jit.load(str(root/'actor.ts'), map_location=device).eval()
        elif backend == 'onnx':
            import onnxruntime as ort
            provider = {'cpu': 'CPUExecutionProvider', 'cuda': 'CUDAExecutionProvider'}[device]
            if provider not in ort.get_available_providers():
                raise Fault('Requested ONNX provider unavailable')
            options = ort.SessionOptions()
            options.intra_op_num_threads = options.inter_op_num_threads = 1
            self.model = ort.InferenceSession(str(root/'actor.onnx'), options, providers=[provider])
            if self.model.get_providers()[0] != provider:
                raise Fault('ONNX provider changed unexpectedly')
        else:
            raise Fault('Unknown inference backend')

    def __call__(self, obs):
        obs = finite(obs, (69,)).astype(np.float32)[None, :]
        if self.backend == 'torchscript':
            with self.torch.inference_mode():
                value = self.model(self.torch.from_numpy(obs).to(self.device)).cpu().numpy()[0]
        else:
            value = self.model.run(['action'], {'observation': obs})[0][0]
        return np.clip(finite(value, (4,)), -1, 1)
