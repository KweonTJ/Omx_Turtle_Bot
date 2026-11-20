// Deliberately no ROS publishers, serial devices or motor-control API.
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

float read(std::istream &s) {
  float x;
  if (!(s >> x) || !std::isfinite(x)) throw std::runtime_error("Invalid numeric input");
  return x;
}
Eigen::MatrixXf matrix(std::istream &s, int rows, int cols) {
  Eigen::MatrixXf m(rows, cols);
  for (int r = 0; r < rows; ++r) for (int c = 0; c < cols; ++c) m(r,c) = read(s);
  return m;
}
void end(std::istream &s) {
  std::string extra;
  if (s >> extra) throw std::runtime_error("Unexpected trailing data");
}
int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::runtime_error("Usage: policy_probe_cpp MODEL_DIRECTORY");
    const std::string path(argv[1]);
    std::ifstream weights(path + "/actor_weights.txt");
    std::string magic;
    int n, h1, h2, a;
    if (!(weights >> magic >> n >> h1 >> h2 >> a) || magic != "OMXV6A01" ||
        n != 69 || h1 != 128 || h2 != 128 || a != 4)
      throw std::runtime_error("Wrong v6 actor contract");
    const auto w1 = matrix(weights, h1, n), b1 = matrix(weights, h1, 1);
    const auto w2 = matrix(weights, h2, h1), b2 = matrix(weights, h2, 1);
    const auto w3 = matrix(weights, a, h2), b3 = matrix(weights, a, 1);
    end(weights);
    auto predict = [&](const Eigen::VectorXf &x) -> Eigen::VectorXf {
      Eigen::VectorXf y = (w1*x+b1).array().tanh();
      y = (w2*y+b2).array().tanh();
      return (w3*y+b3).array().max(-1.f).min(1.f);
    };
    std::ifstream fixture(path + "/parity_vectors.txt");
    int count;
    if (!(fixture >> magic >> count) || magic != "OMXV6TEST01" || count < 1 || count > 100000)
      throw std::runtime_error("Invalid parity fixture");
    float error = 0;
    Eigen::VectorXf x(n);
    for (int i=0; i<count; ++i) {
      x = matrix(fixture,n,1);
      const Eigen::VectorXf expected = matrix(fixture,a,1), actual = predict(x);
      if (!actual.allFinite()) throw std::runtime_error("Non-finite actor output");
      error = std::max(error,(actual-expected).cwiseAbs().maxCoeff());
    }
    end(fixture);
    if (error > 1e-5f) throw std::runtime_error("Actor parity failed: " + std::to_string(error));
    volatile float sink = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int i=0;i<2000;++i) sink = predict(x)[0];
    (void)sink;
    const double us = std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count()/2000.;
    std::cout << "{\"backend\":\"cpp_eigen_cpu\",\"samples\":" << count
      << ",\"max_abs_error\":" << error << ",\"mean_inference_us\":" << us
      << ",\"motor_commands_sent\":0,\"hardware_motion_validated\":false}\n";
    return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
