#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

#include "AccelerationConstraint.hpp"

namespace
{
constexpr float GRAVITY = 9.80665f;
constexpr float WEIGHT = 182.692f;
constexpr float MASS = WEIGHT / GRAVITY;
constexpr float RADIUS = 0.06f;
constexpr float CENTER_RADIUS = 0.233f;
constexpr float INERTIA = 0.49436791247f;
constexpr float MU = 0.35f;
constexpr float REDUCTION = 14.0f;
constexpr float CURRENT_PER_TORQUE = 52437.5f / REDUCTION;
constexpr float KT = 1.99688994e-6f;
constexpr float K1 = 2.0e-7f;
constexpr float K2 = 3.0e-7f;
constexpr float PI = 3.14159265358979323846f;

struct Scenario
{
  float ax = 0.0f;
  float ay = 0.0f;
  float alpha = 0.0f;
  float inertia = INERTIA;
  float power_budget = 100.0f - 5.5f - 4.0f;
  float angular_weight = 1.0f;
  std::array<float, 4> steer{};
  std::array<float, 4> pid{};
  std::array<float, 4> rpm{};
};

struct Model
{
  AccelerationConstraint::Input input{};
  std::array<float, 4> translation{};
  std::array<float, 4> rotation{};
  float pid_scale = 1.0f;
};

void near(double actual, double expected, double tolerance = 1e-5)
{
  assert(std::isfinite(actual));
  if (std::abs(actual - expected) > tolerance)
  {
    std::fprintf(stderr, "Expected %.12g, got %.12g\n", expected, actual);
    assert(false);
  }
}

float wheel_x(int index)
{
  return CENTER_RADIUS * (index == 0 || index == 3 ? -1.0f : 1.0f) / std::sqrt(2.0f);
}

float wheel_y(int index)
{
  return CENTER_RADIUS * (index < 2 ? -1.0f : 1.0f) / std::sqrt(2.0f);
}

float effective_alpha(const Scenario& scenario)
{
  return scenario.inertia > 0.0f ? scenario.alpha : 0.0f;
}

float torque_limit()
{
  return std::min({4.4f, 16384.0f / CURRENT_PER_TORQUE, MU * WEIGHT * RADIUS / 4.0f});
}

// Physical scenario adapter independent of LibXR, motor drivers and RTOS objects.
Model make_model(const Scenario& scenario, float pid_scale)
{
  Model model;
  model.pid_scale = pid_scale;
  auto& input = model.input;
  const float ALPHA = effective_alpha(scenario);
  const float ACCELERATION = std::hypot(scenario.ax, scenario.ay);
  input.power_budget = scenario.power_budget;
  input.weights = {ACCELERATION > 1e-6f ? 1.0f : 0.0f,
                   std::abs(ALPHA) > 1e-6f ? scenario.angular_weight : 0.0f};
  input.halfplanes[input.halfplane_count++] = {
      ACCELERATION / (MU * GRAVITY),
      std::abs(ALPHA) * scenario.inertia / (MU * WEIGHT * CENTER_RADIUS), 1.0f};
  for (int i = 0; i < 4; ++i)
  {
    const float SIN = std::sin(scenario.steer[i]);
    const float COS = std::cos(scenario.steer[i]);
    model.translation[i] = RADIUS * MASS * (scenario.ax * SIN + scenario.ay * COS) / 4.0f;
    model.rotation[i] = RADIUS * scenario.inertia * ALPHA *
                        (-wheel_y(i) * SIN + wheel_x(i) * COS) /
                        (4.0f * CENTER_RADIUS * CENTER_RADIUS);
    auto& motor = input.motors[i];
    motor = {model.translation[i] * CURRENT_PER_TORQUE,
             model.rotation[i] * CURRENT_PER_TORQUE,
             pid_scale * scenario.pid[i] * CURRENT_PER_TORQUE,
             scenario.rpm[i],
             KT,
             K1,
             K2};
    const float CURRENT_LIMIT = torque_limit() * CURRENT_PER_TORQUE;
    input.halfplanes[input.halfplane_count++] = {motor.u, motor.v,
                                                 CURRENT_LIMIT - motor.c};
    input.halfplanes[input.halfplane_count++] = {-motor.u, -motor.v,
                                                 CURRENT_LIMIT + motor.c};
  }
  assert(input.halfplane_count == 9);
  return model;
}

// Reconstruct exactly the float torque/current path sent to Motor::Control.
float output_torque(const Scenario& scenario, const Model& model, int index,
                    float translation_fraction, float angular_fraction)
{
  const float FF = translation_fraction * model.translation[index] +
                   angular_fraction * model.rotation[index];
  return model.pid_scale * scenario.pid[index] + FF;
}

double output_power(const Scenario& scenario, const Model& model, float x, float y,
                    bool clip_negative = false)
{
  double sum = 0.0;
  for (int i = 0; i < 4; ++i)
  {
    const float CURRENT = output_torque(scenario, model, i, x, y) * CURRENT_PER_TORQUE;
    const double POWER = static_cast<double>(KT) * CURRENT * scenario.rpm[i] +
                         static_cast<double>(K1) * CURRENT * CURRENT +
                         static_cast<double>(K2) * scenario.rpm[i] * scenario.rpm[i];
    sum += clip_negative ? std::max(0.0, POWER) : POWER;
  }
  return sum;
}

bool physically_feasible(const Scenario& scenario, const Model& model, float x, float y)
{
  const double TRANSLATION = std::hypot(scenario.ax, scenario.ay) * x;
  const double ROTATION = std::abs(effective_alpha(scenario)) * y;
  if (TRANSLATION / (MU * GRAVITY) +
          ROTATION * scenario.inertia / (MU * WEIGHT * CENTER_RADIUS) >
      1.0)
  {
    return false;
  }
  for (int i = 0; i < 4; ++i)
  {
    if (std::abs(output_torque(scenario, model, i, x, y)) > torque_limit())
    {
      return false;
    }
  }
  return output_power(scenario, model, x, y) <= scenario.power_budget;
}

void check_result(const Scenario& scenario, const Model& model,
                  const AccelerationConstraint::Result& result)
{
  assert(result.feasible);
  const float X = result.point.x;
  const float Y = result.point.y;
  assert(X >= 0.0f && X <= 1.0f && Y >= 0.0f && Y <= 1.0f);
  const double DIAMOND = std::hypot(scenario.ax, scenario.ay) * X / (MU * GRAVITY) +
                         std::abs(effective_alpha(scenario)) * Y * scenario.inertia /
                             (MU * WEIGHT * CENTER_RADIUS);
  assert(DIAMOND <= 1.0 + 3e-6);
  for (int i = 0; i < 4; ++i)
  {
    const float TORQUE = output_torque(scenario, model, i, X, Y);
    assert(std::abs(TORQUE) <= torque_limit() + 5e-6f);
    assert(std::abs(TORQUE * CURRENT_PER_TORQUE) <= 16384.0f + 0.02f);
    const auto& MOTOR = model.input.motors[i];
    const float AFFINE_CURRENT = MOTOR.u * X + MOTOR.v * Y + MOTOR.c;
    near(TORQUE * CURRENT_PER_TORQUE, AFFINE_CURRENT, 0.005);
  }
  // Separate float reconstruction may differ slightly from the solver's double
  // evaluation.
  assert(output_power(scenario, model, X, Y) <= scenario.power_budget + 3e-4);
}

void feedforward_equivalence()
{
  std::mt19937 random(0xff3508U);
  std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
  for (int trial = 0; trial < 5000; ++trial)
  {
    Scenario scenario;
    scenario.ax = 8.0f * unit(random);
    scenario.ay = 8.0f * unit(random);
    scenario.alpha = 80.0f * unit(random);
    for (float& angle : scenario.steer)
    {
      angle = PI * unit(random);
    }
    const Model MODEL = make_model(scenario, 1.0f);
    for (int i = 0; i < 4; ++i)
    {
      // Original force allocation, projected only after forming its 2D force vector.
      const double SCALE =
          scenario.inertia * scenario.alpha / (4.0 * CENTER_RADIUS * CENTER_RADIUS);
      const double FX = MASS * scenario.ax / 4.0 - SCALE * wheel_y(i);
      const double FY = MASS * scenario.ay / 4.0 + SCALE * wheel_x(i);
      const double OLD_TORQUE =
          RADIUS * (FX * std::sin(scenario.steer[i]) + FY * std::cos(scenario.steer[i]));
      near(MODEL.translation[i] + MODEL.rotation[i], OLD_TORQUE, 3e-6);
    }
  }
}

void deterministic()
{
  Scenario scenario;
  scenario.ay = 1.5f;
  scenario.rpm = {5000.0f, -3000.0f, -3000.0f, -3000.0f};
  scenario.power_budget = 10.0f;
  Model model = make_model(scenario, 1.0f);
  auto result = AccelerationConstraint::Solve(model.input);
  check_result(scenario, model, result);
  near(result.point.x, 1.0);
  assert(output_power(scenario, model, result.point.x, result.point.y, true) >
         scenario.power_budget);

  // A shared physical acceleration budget must favor rotation more under ROTOR weight.
  scenario = {};
  scenario.ay = 3.0f;
  scenario.alpha = 30.0f;
  scenario.power_budget = 500.0f;
  model = make_model(scenario, 1.0f);
  const auto BALANCED = AccelerationConstraint::Solve(model.input);
  check_result(scenario, model, BALANCED);
  scenario.angular_weight = 4.0f;
  model = make_model(scenario, 1.0f);
  const auto ROTOR = AccelerationConstraint::Solve(model.input);
  check_result(scenario, model, ROTOR);
  assert(ROTOR.point.y > BALANCED.point.y + 0.5f);
  assert(ROTOR.point.x < BALANCED.point.x);

  // Opposite angular requests must give opposite torques with symmetric budgets.
  scenario = {};
  scenario.alpha = 40.0f;
  model = make_model(scenario, 1.0f);
  const auto POSITIVE = AccelerationConstraint::Solve(model.input);
  check_result(scenario, model, POSITIVE);
  scenario.alpha = -40.0f;
  const Model NEGATIVE_MODEL = make_model(scenario, 1.0f);
  const auto NEGATIVE = AccelerationConstraint::Solve(NEGATIVE_MODEL.input);
  check_result(scenario, NEGATIVE_MODEL, NEGATIVE);
  near(POSITIVE.point.y, NEGATIVE.point.y);
  for (int i = 0; i < 4; ++i)
  {
    near(output_torque(scenario, model, i, POSITIVE.point.x, POSITIVE.point.y),
         -output_torque(scenario, NEGATIVE_MODEL, i, NEGATIVE.point.x, NEGATIVE.point.y));
  }

  // Zero axes and disabled yaw inertia leave no spurious rotational constraint/output.
  scenario = {};
  model = make_model(scenario, 1.0f);
  result = AccelerationConstraint::Solve(model.input);
  check_result(scenario, model, result);
  assert(model.input.weights.x == 0.0f && model.input.weights.y == 0.0f);
  scenario.ay = 2.0f;
  scenario.alpha = 100.0f;
  scenario.inertia = 0.0f;
  model = make_model(scenario, 1.0f);
  result = AccelerationConstraint::Solve(model.input);
  check_result(scenario, model, result);
  assert(model.input.weights.y == 0.0f && model.input.halfplanes[0].y == 0.0f);
  for (float rotation : model.rotation)
  {
    near(rotation, 0.0);
  }
}

void pid_fallbacks()
{
  for (const float REQUESTED_PID : {0.2f, 1.2f, 2.5f})
  {
    Scenario scenario;
    scenario.pid.fill(REQUESTED_PID);
    const auto FROZEN_PID = scenario.pid;
    float accepted = -1.0f;
    for (const float SCALE : {1.0f, 0.5f, 0.0f})
    {
      const Model MODEL = make_model(scenario, SCALE);
      const auto RESULT = AccelerationConstraint::Solve(MODEL.input);
      assert(scenario.pid == FROZEN_PID);
      if (RESULT.feasible)
      {
        check_result(scenario, MODEL, RESULT);
        accepted = SCALE;
        break;
      }
    }
    near(accepted, REQUESTED_PID == 0.2f ? 1.0 : REQUESTED_PID == 1.2f ? 0.5 : 0.0);
  }
  Scenario impossible;
  impossible.rpm.fill(9000.0f);
  impossible.power_budget = 20.0f;
  for (const float SCALE : {1.0f, 0.5f, 0.0f})
  {
    const auto RESULT =
        AccelerationConstraint::Solve(make_model(impossible, SCALE).input);
    assert(!RESULT.feasible &&
           RESULT.status == AccelerationConstraint::Status::INFEASIBLE);
  }
}

void random_physical_scenarios()
{
  std::mt19937 random(0xc0ffeeU);
  std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
  std::uniform_real_distribution<float> positive(0.0f, 1.0f);
  int feasible_count = 0;
  for (int trial = 0; trial < 600; ++trial)
  {
    Scenario scenario;
    scenario.ax = 8.0f * unit(random);
    scenario.ay = 8.0f * unit(random);
    scenario.alpha = 80.0f * unit(random);
    scenario.angular_weight = trial % 2 ? 4.0f : 1.0f;
    scenario.power_budget = 5.0f + 90.0f * positive(random);
    for (int i = 0; i < 4; ++i)
    {
      scenario.steer[i] = PI * unit(random);
      scenario.rpm[i] = 5000.0f * unit(random);
      scenario.pid[i] = 2.0f * unit(random);
    }
    for (const float SCALE : {1.0f, 0.5f, 0.0f})
    {
      const Model MODEL = make_model(scenario, SCALE);
      const auto RESULT = AccelerationConstraint::Solve(MODEL.input);
      double reference = -std::numeric_limits<double>::infinity();
      constexpr int GRID = 40;
      for (int ix = 0; ix <= GRID; ++ix)
      {
        for (int iy = 0; iy <= GRID; ++iy)
        {
          const float X = static_cast<float>(ix) / GRID;
          const float Y = static_cast<float>(iy) / GRID;
          if (physically_feasible(scenario, MODEL, X, Y))
          {
            reference =
                std::max(reference, static_cast<double>(MODEL.input.weights.x) * X +
                                        static_cast<double>(MODEL.input.weights.y) * Y);
          }
        }
      }
      if (std::isfinite(reference))
      {
        assert(RESULT.feasible);
        const double OBJECTIVE = MODEL.input.weights.x * RESULT.point.x +
                                 MODEL.input.weights.y * RESULT.point.y;
        assert(OBJECTIVE >= reference - 3e-4);
      }
      if (RESULT.feasible)
      {
        check_result(scenario, MODEL, RESULT);
        ++feasible_count;
        break;
      }
    }
  }
  assert(feasible_count > 400);
}
}  // namespace

int main()
{
  feedforward_equivalence();
  deterministic();
  pid_fallbacks();
  random_physical_scenarios();
  std::puts(
      "PASS: 5000 FF equivalence cases, physical torque/current/grip/power bounds, "
      "signed braking, +/- yaw, zero axes, J=0, ROTOR weighting, frozen PID "
      "fallbacks and 600 dense-reference physical scenarios.");
}
