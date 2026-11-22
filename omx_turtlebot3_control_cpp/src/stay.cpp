#include "omx_turtlebot3_control_cpp/control.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace omx_v6 {
namespace {
void require(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }
double positive(const YAML::Node &node) {
  const double value = node.as<double>();
  require(std::isfinite(value) && value > 0, "Invalid Stay startup configuration");
  return value;
}
}
StayTrajectory::StayTrajectory(const Config &config, const Sample &sample,
                               const Joints &goal, double stamp)
 : start_(sample.q), goal_(goal), started_(stamp), previous_(stamp), max_cycle_(config.max_cycle) {
  const auto c = config.data["control"];
  duration_ = positive(c["stay_min_duration_s"]);
  timeout_ = positive(c["stay_timeout_s"]);
  position_tolerance_ = positive(c["stay_position_tolerance_rad"]);
  velocity_tolerance_ = positive(c["stay_velocity_tolerance_rad_s"]);
  stable_s_ = positive(c["stable_s"]);
  require(duration_ >= 3 && position_tolerance_ <= c["start_stay_tolerance_rad"].as<double>(),
          "Stay startup must respect the mission arrival tolerance and minimum duration");
  require(std::isfinite(stamp) && stamp >= sample.stamp && stamp-sample.stamp <= max_cycle_,
          "Stay startup requires fresh feedback");
  for (size_t i = 0; i < 4; ++i) {
    require(std::isfinite(start_[i]) && std::isfinite(goal_[i]) &&
            start_[i] >= config.low[i] && start_[i] <= config.high[i] &&
            goal_[i] >= config.low[i] && goal_[i] <= config.high[i], "Stay path outside URDF limits");
    require(std::isfinite(sample.dq[i]) && std::abs(sample.dq[i]) <= velocity_tolerance_,
            "Stay startup requires stationary joints");
    const double distance = std::abs(goal_[i]-start_[i]);
    // max(ds/du)=1.875; max(abs(d2s/du2))=10/sqrt(3).
    duration_ = std::max({duration_, 1.875*distance/config.velocity,
                         std::sqrt((10/std::sqrt(3.))*distance/config.acceleration)});
  }
  controller_trajectory(start_, start_);
  controller_trajectory(goal_, goal_);
  require(timeout_ > duration_+stable_s_+max_cycle_, "Stay timeout shorter than the rate-limited trajectory");
}
void StayTrajectory::restart_clock(double stamp) {
  started_ = previous_ = stamp;
  settled_since_ = -1; ready_ = false;
}
Joints StayTrajectory::update(const Sample &sample, double stamp) {
  require(std::isfinite(stamp) && stamp > previous_ && stamp-previous_ <= max_cycle_ &&
          stamp >= sample.stamp && stamp-sample.stamp <= max_cycle_, "Stay feedback/control deadline missed");
  require(ready_ || stamp-started_ <= timeout_, "Stay arrival timeout");
  previous_ = stamp;
  const double u = std::clamp((stamp-started_)/duration_, 0., 1.);
  const double s = u*u*u*(10+u*(-15+6*u));
  Joints command;
  bool settled = u == 1.;
  for (size_t i = 0; i < 4; ++i) {
    require(std::isfinite(sample.q[i]) && std::isfinite(sample.dq[i]), "Invalid Stay feedback");
    command[i] = start_[i]+(goal_[i]-start_[i])*s;
    settled &= std::abs(sample.q[i]-goal_[i]) <= position_tolerance_ &&
               std::abs(sample.dq[i]) <= velocity_tolerance_;
  }
  if (!settled) settled_since_ = -1;
  else if (settled_since_ < 0) settled_since_ = stamp;
  ready_ = settled && stamp-settled_since_ >= stable_s_;
  return command;
}
}  // namespace omx_v6
