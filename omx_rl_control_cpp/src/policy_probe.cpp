// Copyright 2026 KweonTJ

#include "omx_rl_control_cpp/policy_actor.hpp"

#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char ** argv)
{
  bool use_torchscript = false;
  std::string device = "auto";
  int model_index = 1;
  if (argc >= 4 && std::string(argv[1]) == "--torchscript") {
    use_torchscript = true;
    device = argv[2];
    model_index = 3;
  }
  if (argc != model_index + 1 + omx_rl_control_cpp::kObservationSize) {
    std::cerr <<
      "usage: policy_probe [--torchscript auto|cpu|cuda] ACTOR_FILE OBSERVATION[33]" <<
      std::endl;
    return 2;
  }
  try {
    omx_rl_control_cpp::PolicyActor actor;
    std::string error;
    const bool loaded = use_torchscript ?
      actor.loadTorchScript(argv[model_index], device, &error) :
      actor.load(argv[model_index], &error);
    if (!loaded) {
      throw std::runtime_error(error);
    }
    omx_rl_control_cpp::Observation observation;
    for (int index = 0; index < omx_rl_control_cpp::kObservationSize; ++index) {
      std::size_t parsed = 0;
      const std::string text(argv[index + model_index + 1]);
      observation(index) = std::stod(text, &parsed);
      if (parsed != text.size()) {
        throw std::invalid_argument("invalid observation value: " + text);
      }
    }
    const auto action = actor.predict(observation);
    std::cout << std::setprecision(10);
    for (int index = 0; index < omx_rl_control_cpp::kActionSize; ++index) {
      if (index != 0) {
        std::cout << ' ';
      }
      std::cout << action(index);
    }
    std::cout << '\n';
    return 0;
  } catch (const std::exception & exception) {
    std::cerr << exception.what() << std::endl;
    return 1;
  }
}
