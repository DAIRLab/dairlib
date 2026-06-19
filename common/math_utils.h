#pragma once
#include <random>

/// Blending utilities
enum BLEND_FUNC { kSigmoid, kExponential };

double blend_sigmoid(double t, double tau, double window);

double blend_exp(double t, double tau, double window);

/// Random sampling utility
inline double RandomUniform(double min, double max) {
  static thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> dist(min, max);
  return dist(rng);
}

int FindBin(const double* bins, int n, double x);