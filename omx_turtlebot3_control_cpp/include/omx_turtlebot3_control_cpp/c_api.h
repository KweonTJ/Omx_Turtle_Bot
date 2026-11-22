#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* All calls catch C++ exceptions. Errors are thread-local; no exception crosses ABI. */
const char *omx_v6_error(void);
void *omx_v6_actor_create(const char *model, const char *config, const char *urdf);
void omx_v6_actor_destroy(void *handle);
int omx_v6_infer(void *handle, const float observation[69], double action[4]);
/* input: target4 goal4 measured4 velocity4 low4 high4 finger finger_goal dt */
int omx_v6_step(void *handle, const float observation[69], const double input[27],
                int phase, int waypoint, int count, double output[13]);
void *omx_v6_transport_create(const char *config, const char *urdf, int execute);
void omx_v6_transport_destroy(void *handle);
/* stamp q4 dq4 finger finger_velocity current voltage wheel2 millis */
int omx_v6_read(void *handle, double sample[16]);
int omx_v6_arm(void *handle);
int omx_v6_begin_stay(void *handle, const double goal[4]);
/* ready is 1 only after measured position AND velocity settle. */
int omx_v6_stay_step(void *handle, int *ready);
int omx_v6_finish_stay(void *handle);
int omx_v6_torque_enabled(void *handle);
int omx_v6_command(void *handle, const double q[4], double finger);
int omx_v6_hold(void *handle);
uint64_t omx_v6_writes(void *handle);
#ifdef __cplusplus
}
#endif
