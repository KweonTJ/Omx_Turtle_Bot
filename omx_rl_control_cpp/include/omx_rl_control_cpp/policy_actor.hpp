// Copyright 2026 KweonTJ

#ifndef OMX_RL_CONTROL_CPP__POLICY_ACTOR_HPP_
#define OMX_RL_CONTROL_CPP__POLICY_ACTOR_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <string>

namespace omx_rl_control_cpp
{

constexpr int kObservationSize = 33;
constexpr int kActionSize = 4;

using Observation = Eigen::Matrix<double, kObservationSize, 1>;
using Action = Eigen::Matrix<double, kActionSize, 1>;

class PolicyActor
{
public:
  bool load(const std::string & path, std::string * error = nullptr);
  Action predict(const Observation & observation) const;
  bool ready() const noexcept {return ready_;}
  const std::string & modelPath() const noexcept {return model_path_;}

private:
  Eigen::MatrixXf weight_1_;
  Eigen::VectorXf bias_1_;
  Eigen::MatrixXf weight_2_;
  Eigen::VectorXf bias_2_;
  Eigen::MatrixXf weight_out_;
  Eigen::VectorXf bias_out_;
  std::string model_path_;
  bool ready_{false};
};

}  // namespace omx_rl_control_cpp

#endif  // OMX_RL_CONTROL_CPP__POLICY_ACTOR_HPP_
