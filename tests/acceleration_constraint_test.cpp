#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

#include "AccelerationConstraint.hpp"

using AccelerationConstraint::Input;
using AccelerationConstraint::Result;
using AccelerationConstraint::Solve;
using AccelerationConstraint::Status;

namespace
{
void near(float actual, float expected, float tolerance = 1e-4f)
{
  assert(std::isfinite(actual));
  if (std::abs(actual - expected) > tolerance)
  {
    std::fprintf(stderr, "Expected %.9g, got %.9g\n", expected, actual);
    assert(false);
  }
}

double power(const Input& input, double x, double y)
{
  double result = 0.0;
  for (const auto& MOTOR : input.motors)
  {
    const double CURRENT = MOTOR.u * x + MOTOR.v * y + MOTOR.c;
    result += MOTOR.k1 * CURRENT * CURRENT +
              static_cast<double>(MOTOR.kt) * MOTOR.rpm * CURRENT +
              static_cast<double>(MOTOR.k2) * MOTOR.rpm * MOTOR.rpm;
  }
  return result;
}

void check_feasible(const Input& input, const Result& result)
{
  assert(result.feasible && result.status == Status::OPTIMAL);
  assert(result.point.x >= 0.0f && result.point.x <= 1.0f);
  assert(result.point.y >= 0.0f && result.point.y <= 1.0f);
  assert(power(input, result.point.x, result.point.y) <= input.power_budget + 1e-4);
  for (int i = 0; i < input.halfplane_count; ++i)
  {
    const auto& PLANE = input.halfplanes[i];
    assert(PLANE.x * result.point.x + PLANE.y * result.point.y <= PLANE.limit + 1e-5f);
  }
}

void deterministic()
{
  Input input;
  input.motors[0].u = 1.0f;
  input.motors[0].k1 = 1.0f;
  input.motors[1].v = 1.0f;
  input.motors[1].k1 = 1.0f;
  input.power_budget = 0.5f;
  Result result = Solve(input);
  check_feasible(input, result);
  near(result.point.x, 0.5f);
  near(result.point.y, 0.5f);

  // A shifted ellipse has an interior support point rather than a polygon-edge optimum.
  input.motors[0].c = -0.4f;
  input.motors[1].c = -0.6f;
  input.motors[1].v = 2.0f;
  input.power_budget = 0.1f;
  result = Solve(input);
  check_feasible(input, result);
  near(result.point.x, 0.4f + std::sqrt(0.1f / 1.25f));
  near(result.point.y, 0.3f + 0.25f * std::sqrt(0.1f / 1.25f));

  // Zero budget may leave exactly one interior feasible point.
  input.power_budget = 0.0f;
  result = Solve(input);
  check_feasible(input, result);
  near(result.point.x, 0.4f);
  near(result.point.y, 0.3f);

  // Rank-one strip, followed by the zero-width line degeneracy.
  input.motors[1] = {};
  input.power_budget = 0.04f;
  result = Solve(input);
  check_feasible(input, result);
  near(result.point.x, 0.6f);
  near(result.point.y, 1.0f);
  input.power_budget = 0.0f;
  result = Solve(input);
  check_feasible(input, result);
  near(result.point.x, 0.4f);
  near(result.point.y, 1.0f);

  // Rank-one parabola x^2 + y <= 0.5: its support is not on a polygon edge.
  input = {};
  input.motors[0].u = 1.0f;
  input.motors[0].k1 = 1.0f;
  input.motors[1].v = 1.0f;
  input.motors[1].rpm = 1.0f;
  input.motors[1].kt = 1.0f;
  input.power_budget = 0.5f;
  result = Solve(input);
  check_feasible(input, result);
  near(result.point.x, 0.5f);
  near(result.point.y, 0.25f);

  // Signed braking power increases usable positive motor power, without abs/max0.
  input = {};
  input.motors[0].u = 140.0f;
  input.motors[0].rpm = 1.0f;
  input.motors[0].kt = 1.0f;
  input.motors[1].v = -40.0f;
  input.motors[1].rpm = 1.0f;
  input.motors[1].kt = 1.0f;
  input.power_budget = 70.0f;
  result = Solve(input);
  check_feasible(input, result);
  near(result.point.x, 110.0f / 140.0f);
  near(result.point.y, 1.0f);

  // Linear clipping, angular preference and a disabled translation axis.
  input = {};
  input.power_budget = 100.0f;
  input.halfplanes[0] = {1.0f, 1.0f, 0.6f};
  input.halfplane_count = 1;
  input.weights = {1.0f, 10.0f};
  result = Solve(input);
  check_feasible(input, result);
  near(result.point.x, 0.0f);
  near(result.point.y, 0.6f);
  input.halfplanes[1] = {1.0f, 0.0f, 0.0f};
  input.halfplane_count = 2;
  result = Solve(input);
  check_feasible(input, result);
  near(result.point.x, 0.0f);

  // Infeasible power still returns the actual minimum-power point on the polygon.
  input = {};
  input.motors[0].u = 1.0f;
  input.motors[0].c = -0.3f;
  input.motors[0].k1 = 1.0f;
  input.motors[1].v = 1.0f;
  input.motors[1].rpm = 1.0f;
  input.motors[1].kt = 1.0f;
  input.motors[2].rpm = 1.0f;
  input.motors[2].k2 = 10.0f;
  input.power_budget = 9.0f;
  result = Solve(input);
  assert(!result.feasible && result.minimum_available &&
         result.status == Status::INFEASIBLE);
  near(result.point.x, 0.3f);
  near(result.point.y, 0.0f);
  near(result.power, 10.0f);

  // Empty geometric polygon must not pretend that the origin is a valid fallback.
  input.halfplanes[0] = {0.0f, 0.0f, -1.0f};
  input.halfplane_count = 1;
  result = Solve(input);
  assert(!result.feasible && !result.minimum_available &&
         result.status == Status::INFEASIBLE);

  input = {};
  input.power_budget = std::numeric_limits<float>::quiet_NaN();
  assert(Solve(input).status == Status::INVALID_INPUT);
  input.power_budget = 1.0f;
  input.motors[0].k1 = -1.0f;
  assert(Solve(input).status == Status::INVALID_INPUT);
}

void randomized()
{
  std::mt19937 random(0xacc35U);
  std::uniform_real_distribution<float> signed_unit(-1.0f, 1.0f);
  std::uniform_real_distribution<float> unit(0.0f, 1.0f);
  for (int trial = 0; trial < 1500; ++trial)
  {
    Input input;
    for (auto& motor : input.motors)
    {
      motor.u = 5000.0f * signed_unit(random);
      motor.v = 5000.0f * signed_unit(random);
      motor.c = 1000.0f * signed_unit(random);
      motor.rpm = 4000.0f * signed_unit(random);
      motor.kt = 1.99688994e-6f;
      motor.k1 = 2e-7f;
      motor.k2 = 3e-7f;
    }
    input.power_budget = 60.0f * unit(random);
    input.weights = {0.1f + unit(random), 0.1f + 5.0f * unit(random)};
    input.halfplanes[0] = {1.0f, 0.7f, 0.2f + 1.5f * unit(random)};
    input.halfplane_count = 1;
    const Result RESULT = Solve(input);
    if (RESULT.feasible)
    {
      check_feasible(input, RESULT);
    }
    double reference = -std::numeric_limits<double>::infinity();
    constexpr int GRID = 80;
    for (int ix = 0; ix <= GRID; ++ix)
    {
      for (int iy = 0; iy <= GRID; ++iy)
      {
        const double X = static_cast<double>(ix) / GRID;
        const double Y = static_cast<double>(iy) / GRID;
        if (X + 0.7 * Y <= input.halfplanes[0].limit &&
            power(input, X, Y) <= input.power_budget)
        {
          reference = std::max(reference, input.weights.x * X + input.weights.y * Y);
        }
      }
    }
    if (std::isfinite(reference))
    {
      if (!RESULT.feasible)
      {
        std::fprintf(stderr, "False infeasible at trial %d\n", trial);
        assert(false);
      }
      const double OBJECTIVE =
          input.weights.x * RESULT.point.x + input.weights.y * RESULT.point.y;
      if (OBJECTIVE + 2e-4 < reference)
      {
        std::fprintf(stderr, "Suboptimal trial %d: %.9g vs grid %.9g\n", trial, OBJECTIVE,
                     reference);
        assert(false);
      }
    }
  }
}
}  // namespace

int main()
{
  deterministic();
  const auto START = std::chrono::steady_clock::now();
  randomized();
  const auto STOP = std::chrono::steady_clock::now();
  std::printf(
      "PASS: polygon/ellipse geometry, signed braking, degenerate quadratics, "
      "infeasible fallback and 1500 dense-reference randomized cases (%.3f s).\n",
      std::chrono::duration<double>(STOP - START).count());
}
