// Copyright 2026 KweonTJ

#include "omx_rl_control_cpp/policy_actor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace omx_rl_control_cpp
{
namespace
{

constexpr std::array<char, 8> kMagic{{'O', 'M', 'X', 'A', 'C', 'T', 'R', '1'}};
constexpr std::uint32_t kFormatVersion = 1;
constexpr std::uint32_t kHiddenSize = 256;

std::uint32_t readU32(std::ifstream & stream)
{
  std::array<unsigned char, 4> bytes{};
  stream.read(reinterpret_cast<char *>(bytes.data()), bytes.size());
  if (!stream) {
    throw std::runtime_error("actor file ended while reading its header");
  }
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

float readFloat32(std::ifstream & stream)
{
  static_assert(sizeof(float) == sizeof(std::uint32_t));
  const std::uint32_t bits = readU32(stream);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  if (!std::isfinite(value)) {
    throw std::runtime_error("actor file contains a non-finite weight");
  }
  return value;
}

void readMatrix(std::ifstream & stream, Eigen::MatrixXf & matrix)
{
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      matrix(row, column) = readFloat32(stream);
    }
  }
}

void readVector(std::ifstream & stream, Eigen::VectorXf & vector)
{
  for (Eigen::Index index = 0; index < vector.size(); ++index) {
    vector(index) = readFloat32(stream);
  }
}

}  // namespace

bool PolicyActor::load(const std::string & path, std::string * error)
{
  ready_ = false;
  model_path_.clear();
  try {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
      throw std::runtime_error("cannot open actor file: " + path);
    }

    std::array<char, 8> magic{};
    stream.read(magic.data(), magic.size());
    if (!stream || magic != kMagic) {
      throw std::runtime_error("invalid actor file magic");
    }

    const auto version = readU32(stream);
    const auto observations = readU32(stream);
    const auto hidden_1 = readU32(stream);
    const auto hidden_2 = readU32(stream);
    const auto actions = readU32(stream);
    if (
      version != kFormatVersion || observations != kObservationSize ||
      hidden_1 != kHiddenSize || hidden_2 != kHiddenSize ||
      actions != kActionSize)
    {
      throw std::runtime_error(
              "unsupported actor dimensions or format version");
    }

    weight_1_.resize(hidden_1, observations);
    bias_1_.resize(hidden_1);
    weight_2_.resize(hidden_2, hidden_1);
    bias_2_.resize(hidden_2);
    weight_out_.resize(actions, hidden_2);
    bias_out_.resize(actions);
    readMatrix(stream, weight_1_);
    readVector(stream, bias_1_);
    readMatrix(stream, weight_2_);
    readVector(stream, bias_2_);
    readMatrix(stream, weight_out_);
    readVector(stream, bias_out_);

    char trailing = 0;
    if (stream.read(&trailing, 1)) {
      throw std::runtime_error("actor file has unexpected trailing data");
    }
    if (!stream.eof()) {
      throw std::runtime_error("failed while validating actor file length");
    }

    model_path_ = path;
    ready_ = true;
    return true;
  } catch (const std::exception & exception) {
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  }
}

Action PolicyActor::predict(const Observation & observation) const
{
  if (!ready_) {
    throw std::runtime_error("PolicyActor::load must succeed before predict");
  }
  if (!observation.allFinite()) {
    throw std::invalid_argument("policy observation contains NaN or Inf");
  }

  const Eigen::VectorXf input = observation.cast<float>();
  const Eigen::VectorXf hidden_1 =
    (weight_1_ * input + bias_1_).array().tanh().matrix();
  const Eigen::VectorXf hidden_2 =
    (weight_2_ * hidden_1 + bias_2_).array().tanh().matrix();
  const Eigen::VectorXf output = weight_out_ * hidden_2 + bias_out_;

  Action action;
  for (int index = 0; index < kActionSize; ++index) {
    action(index) = std::clamp(
      static_cast<double>(output(index)), -1.0, 1.0);
  }
  return action;
}

}  // namespace omx_rl_control_cpp
