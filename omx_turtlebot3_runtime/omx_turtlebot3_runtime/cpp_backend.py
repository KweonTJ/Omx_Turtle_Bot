"""C ABI only: perception/mission stay in Python; control and motor I/O are C++."""
import ctypes as ct
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_prefix
from .core import Fault, RobotSample, finite

F69 = ct.c_float * 69
D4 = ct.c_double * 4
D13 = ct.c_double * 13
D16 = ct.c_double * 16
D27 = ct.c_double * 27


def library(path=None):
    path = Path(path) if path else Path(get_package_prefix('omx_turtlebot3_control_cpp'))/'lib/libomx_v6_control.so'
    lib = ct.CDLL(str(path))
    lib.omx_v6_error.restype = ct.c_char_p
    lib.omx_v6_actor_create.argtypes = [ct.c_char_p]*3
    lib.omx_v6_actor_create.restype = ct.c_void_p
    lib.omx_v6_actor_destroy.argtypes = [ct.c_void_p]
    lib.omx_v6_actor_destroy.restype = None
    lib.omx_v6_infer.argtypes = [ct.c_void_p, F69, D4]
    lib.omx_v6_infer.restype = ct.c_int
    lib.omx_v6_step.argtypes = [ct.c_void_p, F69, D27, ct.c_int, ct.c_int, ct.c_int, D13]
    lib.omx_v6_step.restype = ct.c_int
    lib.omx_v6_transport_create.argtypes = [ct.c_char_p, ct.c_char_p, ct.c_int]
    lib.omx_v6_transport_create.restype = ct.c_void_p
    lib.omx_v6_transport_destroy.argtypes = [ct.c_void_p]
    lib.omx_v6_transport_destroy.restype = None
    lib.omx_v6_read.argtypes = [ct.c_void_p, D16]
    lib.omx_v6_read.restype = ct.c_int
    for name in ('arm', 'hold', 'finish_stay'):
        fn = getattr(lib, 'omx_v6_'+name)
        fn.argtypes, fn.restype = [ct.c_void_p], ct.c_int
    lib.omx_v6_begin_stay.argtypes = [ct.c_void_p, D4]
    lib.omx_v6_begin_stay.restype = ct.c_int
    lib.omx_v6_stay_step.argtypes = [ct.c_void_p, ct.POINTER(ct.c_int)]
    lib.omx_v6_stay_step.restype = ct.c_int
    lib.omx_v6_torque_enabled.argtypes = [ct.c_void_p]
    lib.omx_v6_torque_enabled.restype = ct.c_int
    lib.omx_v6_command.argtypes = [ct.c_void_p, D4, ct.c_double]
    lib.omx_v6_command.restype = ct.c_int
    lib.omx_v6_writes.argtypes = [ct.c_void_p]
    lib.omx_v6_writes.restype = ct.c_uint64
    return lib


def checked(lib, result):
    if result != 0:
        raise Fault(lib.omx_v6_error().decode('utf-8', errors='replace'))


def path_bytes(path):
    return str(Path(path).resolve()).encode()


class NativeActor:
    backend = 'cpp_opencv_dnn_onnx_cpu'

    def __init__(self, model, config, urdf, library_path=None):
        self.handle = None
        self.lib = library(library_path)
        self.handle = self.lib.omx_v6_actor_create(path_bytes(model), path_bytes(config), path_bytes(urdf))
        if not self.handle:
            checked(self.lib, -1)

    def __call__(self, obs):
        out = D4()
        checked(self.lib, self.lib.omx_v6_infer(self.handle, F69(*finite(obs, (69,))), out))
        return np.array(out)

    def control_step(self, obs, target, goal, measured, velocity, phase, waypoint,
                     count, dt, finger, finger_goal, low, high):
        values = np.r_[finite(target, (4,)), finite(goal, (4,)), finite(measured, (4,)),
                       finite(velocity, (4,)), finite(low, (4,)), finite(high, (4,)),
                       finger, finger_goal, dt]
        out = D13()
        checked(self.lib, self.lib.omx_v6_step(self.handle, F69(*finite(obs, (69,))),
                D27(*finite(values, (27,))), int(phase), int(waypoint), int(count), out))
        result = np.array(out)
        return result[:4], result[4:8], result[8:12], float(result[12])

    def close(self):
        if self.handle:
            self.lib.omx_v6_actor_destroy(self.handle)
            self.handle = None

    def __del__(self):
        self.close()


class NativeTransport:
    def __init__(self, cfg, config, urdf, execute=False, library_path=None):
        self.handle, self.last = None, None
        self.execute, self.closed = bool(execute), False
        self.last_trajectory = None
        self._writes = 0
        self.lib = library(library_path)
        self.handle = self.lib.omx_v6_transport_create(path_bytes(config), path_bytes(urdf), int(execute))
        if not self.handle:
            checked(self.lib, -1)
        try:
            self.read()
        except BaseException:
            self.close()
            raise

    @property
    def writes(self):
        return int(self.lib.omx_v6_writes(self.handle)) if self.handle else self._writes

    def read(self):
        out = D16()
        checked(self.lib, self.lib.omx_v6_read(self.handle, out))
        v = np.array(out)
        self.last = RobotSample(float(v[0]), v[1:5], v[5:9], *map(float, v[9:13]),
                                (int(v[13]), int(v[14])), int(v[15]))
        return self.last

    def arm(self):
        checked(self.lib, self.lib.omx_v6_arm(self.handle))

    @property
    def torque_enabled(self):
        return bool(self.lib.omx_v6_torque_enabled(self.handle))

    def begin_stay(self, goal):
        checked(self.lib, self.lib.omx_v6_begin_stay(self.handle, D4(*finite(goal, (4,)))))

    def stay_step(self):
        ready = ct.c_int()
        checked(self.lib, self.lib.omx_v6_stay_step(self.handle, ct.byref(ready)))
        return bool(ready.value)

    def finish_stay(self):
        checked(self.lib, self.lib.omx_v6_finish_stay(self.handle))

    def command(self, q, finger):
        q = finite(q, (4,))
        checked(self.lib, self.lib.omx_v6_command(self.handle, D4(*q), float(finger)))
        raw = q.copy()
        raw[2] = 2*self.last.q[2]-q[2]
        self.last_trajectory = dict(route='cpp_mp_control_equivalent_opencr',
            reference_q=self.last.q.tolist(), raw_q=raw.tolist(), controller_q=q.tolist(),
            joint4_compensated_again=False)

    def hold(self):
        checked(self.lib, self.lib.omx_v6_hold(self.handle))

    def close(self):
        if self.handle:
            # Count hold writes too. The native destructor also has a best-effort hold.
            try:
                self.hold()
            finally:
                self._writes = self.writes
                self.lib.omx_v6_transport_destroy(self.handle)
                self.handle = None
                self.closed = True

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
