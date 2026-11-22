#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace mp_control::level_retraction {
using Joints = std::array<double, 4>;
struct Geometry {double shoulder_x, shoulder_z, forearm, tool;};
struct Pose {double radius, z, pitch;};
struct Point {Joints q; double seconds;};

// URDF chain: Ry(q2) T(x,0,z) Ry(q3) T(L3,0,0) Ry(q4) T(tool,0,0).
// Joint1 is vertical yaw, so neither its angle nor the fixed base origin changes
// the height difference constrained here.
inline Pose forward(const Joints & q, const Geometry & g)
{
  const double a = q[1], b = a + q[2], p = b + q[3];
  return {g.shoulder_x*std::cos(a) + g.shoulder_z*std::sin(a) +
    g.forearm*std::cos(b) + g.tool*std::cos(p),
    -g.shoulder_x*std::sin(a) + g.shoulder_z*std::cos(a) -
    g.forearm*std::sin(b) - g.tool*std::sin(p), p};
}

inline std::vector<Point> plan(
  const Joints & start, double elbow_delta, double duration, int samples,
  const Geometry & g, const Joints & low, const Joints & high)
{
  if (!std::isfinite(elbow_delta) || elbow_delta < 0 || elbow_delta > 1.0 ||
    !std::isfinite(duration) || duration < .2 || samples < 8 || samples > 128 ||
    !std::isfinite(g.shoulder_x) || !std::isfinite(g.shoulder_z) ||
    !std::isfinite(g.forearm) || !std::isfinite(g.tool) ||
    g.shoulder_z <= 0 || g.forearm <= 0 || g.tool <= 0)
    throw std::invalid_argument("Invalid level-retraction configuration");
  for (size_t i = 0; i < 4; ++i)
    if (!std::isfinite(start[i]) || !std::isfinite(low[i]) || !std::isfinite(high[i]) ||
      low[i] >= high[i] || start[i] < low[i] || start[i] > high[i])
      throw std::invalid_argument("Level retraction starts outside motor limits");
  const auto reference = forward(start, g);
  const double wrist_z = reference.z + g.tool*std::sin(reference.pitch);
  constexpr double tau = 6.2831853071795864769;
  Joints previous = start;
  double previous_seconds = 0.0;
  std::vector<Point> result;
  result.reserve(static_cast<size_t>(samples));
  for (int i = 1; i <= samples; ++i) {
    const double t = double(i)/samples;
    const double u = t*t*t*(10.0 + t*(-15.0 + 6.0*t));
    const double elbow = start[2] + elbow_delta*u;
    if (elbow < low[2] || elbow > high[2])
      throw std::runtime_error("Level-retraction elbow target exceeds motor limits");
    const double a = g.shoulder_z - g.forearm*std::sin(elbow);
    const double b = -g.shoulder_x - g.forearm*std::cos(elbow);
    const double length = std::hypot(a, b);
    if (length < 1e-12 || std::abs(wrist_z) > length + 1e-10)
      throw std::runtime_error("No constant-height solution for retraction");
    const double phi = std::atan2(b, a);
    const double angle = std::acos(std::clamp(wrist_z/length, -1.0, 1.0));
    double best = std::numeric_limits<double>::infinity();
    Joints chosen{};
    for (double sign : {-1.0, 1.0}) for (int turn = -1; turn <= 1; ++turn) {
      Joints q{start[0], phi + sign*angle + turn*tau, elbow, 0.0};
      q[3] = reference.pitch - q[1] - q[2];
      if (q[1] < low[1] || q[1] > high[1] || q[3] < low[3] || q[3] > high[3]) continue;
      const double distance = std::abs(q[1]-previous[1]) + std::abs(q[3]-previous[3]);
      if (distance < best) {best = distance; chosen = q;}
    }
    if (!std::isfinite(best))
      throw std::runtime_error("Constant-height retraction cannot satisfy joint limits");
    const double seconds = duration*t;
    for (size_t joint = 0; joint < 4; ++joint)
      if (std::abs(chosen[joint]-previous[joint]) > .9*(seconds-previous_seconds) + 1e-9)
        throw std::runtime_error("Level-retraction branch or speed exceeds trial envelope");
    const auto checked = forward(chosen, g);
    if (std::abs(checked.z-reference.z) > 1e-8 ||
      std::abs(checked.pitch-reference.pitch) > 1e-8)
      throw std::runtime_error("Level-retraction constraint check failed");
    result.push_back({chosen, seconds});
    previous = chosen;
    previous_seconds = seconds;
  }
  return result;
}
}  // namespace mp_control::level_retraction
