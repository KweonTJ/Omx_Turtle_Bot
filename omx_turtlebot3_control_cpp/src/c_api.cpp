#include "omx_turtlebot3_control_cpp/c_api.h"
#include "omx_turtlebot3_control_cpp/control.hpp"
#include <algorithm>
#include <exception>
#include <stdexcept>

namespace {
thread_local std::string error;
template<class F> int call(F f) {
  try { f(); error.clear(); return 0; }
  catch (const std::exception &e) { error = e.what(); return -1; }
  catch (...) { error = "Unknown native failure"; return -1; }
}
struct Controller {
  omx_v6::Config config;
  omx_v6::Actor actor;
  Controller(const char *model, const char *cfg, const char *urdf) : config(cfg, urdf), actor(model) {}
};
omx_v6::Joints joints(const double *v) { omx_v6::Joints out; std::copy_n(v, 4, out.begin()); return out; }
omx_v6::Observation observation(const float *v) { omx_v6::Observation out; std::copy_n(v, 69, out.begin()); return out; }
void valid(const void *p) { if (!p) throw std::runtime_error("Null native argument"); }
}
extern "C" {
const char *omx_v6_error() { return error.c_str(); }
void *omx_v6_actor_create(const char *model, const char *cfg, const char *urdf) {
  Controller *p = nullptr;
  call([&] { valid(model); valid(cfg); valid(urdf); p = new Controller(model, cfg, urdf); }); return p;
}
void omx_v6_actor_destroy(void *p) { delete static_cast<Controller *>(p); }
int omx_v6_infer(void *p, const float *obs, double *out) {
  return call([&] { valid(p); valid(obs); valid(out); const auto a = static_cast<Controller *>(p)->actor.infer(observation(obs)); std::copy(a.begin(), a.end(), out); });
}
int omx_v6_step(void *p, const float *obs, const double *in, int phase, int waypoint, int count, double *out) {
  return call([&] {
    valid(p); valid(obs); valid(in); valid(out); auto &c = *static_cast<Controller *>(p);
    const auto action = c.actor.infer(observation(obs));
    const auto result = omx_v6::step(c.config, joints(in), joints(in+4), joints(in+8), action, joints(in+12), phase, waypoint, count, in[26], in[24], in[25], joints(in+16), joints(in+20));
    std::copy(result.q.begin(), result.q.end(), out);
    std::copy(result.action.begin(), result.action.end(), out+4);
    std::copy(result.velocity.begin(), result.velocity.end(), out+8); out[12] = result.finger;
  });
}
void *omx_v6_transport_create(const char *cfg, const char *urdf, int execute) {
  omx_v6::Transport *p = nullptr;
  call([&] {
    valid(cfg); valid(urdf);
    if (execute != 0 && execute != 1) throw std::runtime_error("Invalid execute flag");
    omx_v6::Config config(cfg, urdf);
    if (execute) config.require_commissioned();  // Before opening any serial device.
    auto bus = omx_v6::sdk_bus(config);
    p = new omx_v6::Transport(std::move(config), execute != 0, std::move(bus));
  }); return p;
}
void omx_v6_transport_destroy(void *p) { delete static_cast<omx_v6::Transport *>(p); }
int omx_v6_read(void *p, double *out) {
  return call([&] {
    valid(p); valid(out); const auto s = static_cast<omx_v6::Transport *>(p)->read();
    out[0] = s.stamp; std::copy(s.q.begin(), s.q.end(), out+1); std::copy(s.dq.begin(), s.dq.end(), out+5);
    out[9] = s.finger; out[10] = s.finger_velocity; out[11] = s.current; out[12] = s.voltage;
    out[13] = s.wheels[0]; out[14] = s.wheels[1]; out[15] = s.millis;
  });
}
int omx_v6_arm(void *p) { return call([&] { valid(p); static_cast<omx_v6::Transport *>(p)->arm(); }); }
int omx_v6_begin_stay(void *p, const double *goal) {
  return call([&] { valid(p); valid(goal); static_cast<omx_v6::Transport *>(p)->begin_stay(joints(goal)); });
}
int omx_v6_stay_step(void *p, int *ready) {
  return call([&] { valid(p); valid(ready); *ready = static_cast<omx_v6::Transport *>(p)->stay_step(); });
}
int omx_v6_finish_stay(void *p) { return call([&] { valid(p); static_cast<omx_v6::Transport *>(p)->finish_stay(); }); }
int omx_v6_torque_enabled(void *p) { return p && static_cast<omx_v6::Transport *>(p)->torque_enabled(); }
int omx_v6_command(void *p, const double *q, double finger) { return call([&] { valid(p); valid(q); static_cast<omx_v6::Transport *>(p)->command(joints(q), finger); }); }
int omx_v6_hold(void *p) { return call([&] { valid(p); static_cast<omx_v6::Transport *>(p)->hold(); }); }
uint64_t omx_v6_writes(void *p) { return p ? static_cast<omx_v6::Transport *>(p)->writes() : 0; }
}
