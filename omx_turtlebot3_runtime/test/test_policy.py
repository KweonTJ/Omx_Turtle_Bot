import os
from pathlib import Path
import numpy as np
import pytest
from omx_turtlebot3_runtime.policy import Actor
from omx_turtlebot3_runtime.core import Fault


def test_exported_model_parity():
    root = os.environ.get('OMX_V6_MODEL_DIR')
    if not root:
        pytest.skip('Set OMX_V6_MODEL_DIR to the immutable exported v6 artifact')
    actor = Actor(root)
    vectors = np.load(Path(root)/'parity_vectors.npz', allow_pickle=False)
    actual = np.array([actor(o) for o in vectors['observations']])
    np.testing.assert_allclose(actual, vectors['actions'], atol=1e-5, rtol=0)
    with pytest.raises(Fault):
        actor(np.zeros(33))
    with pytest.raises(Fault):
        actor(np.full(69, np.nan))
