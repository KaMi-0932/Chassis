#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

/**
 * Two-dimensional acceleration allocation, following the document's polygon,
 * ellipse-support and edge-intersection construction. Motor speeds and wheel PID
 * outputs are frozen in Input. Powers are signed; braking power is not clipped.
 * Decision coordinates are normalized acceleration fractions in [0, 1]. All
 * geometry uses single precision so Cortex-M4F runs it on the hardware FPU.
 */
namespace AccelerationConstraint
{
struct Vec2
{
  float x = 0.0f;
  float y = 0.0f;
};

struct MotorModel
{
  float u = 0.0f;  ///< Current = u*x + v*y + c, in motor command units.
  float v = 0.0f;
  float c = 0.0f;
  float rpm = 0.0f;
  float kt = 0.0f;
  float k1 = 0.0f;
  float k2 = 0.0f;
};

struct HalfPlane
{
  float x = 0.0f;
  float y = 0.0f;
  float limit = 0.0f;  ///< x*decision.x + y*decision.y <= limit.
};

struct Input
{
  MotorModel motors[4]{};
  HalfPlane halfplanes[12]{};
  int halfplane_count = 0;
  Vec2 weights{1.0f, 1.0f};
  float power_budget = 0.0f;  ///< Already excludes static chassis loss and margin.
};

enum class Status
{
  OPTIMAL,
  INFEASIBLE,
  INVALID_INPUT
};

struct Result
{
  bool feasible = false;
  bool minimum_available = false;
  Vec2 point{};
  float power = 0.0f;
  Status status = Status::INVALID_INPUT;
};

namespace Detail
{
struct Point
{
  float x;
  float y;
};

struct Polygon
{
  Point points[20]{};
  int count = 0;
};

struct Quadratic
{
  float a = 0.0f;
  float b = 0.0f;  // f = a*x*x + 2*b*x*y + c*y*y + d*x + e*y + f.
  float c = 0.0f;
  float d = 0.0f;
  float e = 0.0f;
  float f = 0.0f;

  float Value(Point p) const
  {
    return (a * p.x + 2.0f * b * p.y + d) * p.x + (c * p.y + e) * p.y + f;
  }
};

inline bool AddPoint(Polygon& polygon, Point point)
{
  if (polygon.count > 0)
  {
    const Point LAST = polygon.points[polygon.count - 1];
    if (std::abs(point.x - LAST.x) + std::abs(point.y - LAST.y) < 1e-12f)
    {
      return true;
    }
  }
  if (polygon.count == 20)
  {
    return false;
  }
  polygon.points[polygon.count++] = point;
  return true;
}

inline bool Clip(Polygon& polygon, HalfPlane plane)
{
  Polygon output;
  if (polygon.count == 0)
  {
    return true;
  }
  Point previous = polygon.points[polygon.count - 1];
  float previous_distance = plane.x * previous.x + plane.y * previous.y - plane.limit;
  for (int i = 0; i < polygon.count; ++i)
  {
    const Point CURRENT = polygon.points[i];
    const float DISTANCE = plane.x * CURRENT.x + plane.y * CURRENT.y - plane.limit;
    if ((DISTANCE <= 0.0f) != (previous_distance <= 0.0f))
    {
      const float T = previous_distance / (previous_distance - DISTANCE);
      if (!AddPoint(output, {previous.x + T * (CURRENT.x - previous.x),
                             previous.y + T * (CURRENT.y - previous.y)}))
      {
        return false;
      }
    }
    if (DISTANCE <= 0.0f && !AddPoint(output, CURRENT))
    {
      return false;
    }
    previous = CURRENT;
    previous_distance = DISTANCE;
  }
  if (output.count > 1)
  {
    const Point FIRST = output.points[0];
    const Point LAST = output.points[output.count - 1];
    if (std::abs(FIRST.x - LAST.x) + std::abs(FIRST.y - LAST.y) < 1e-12f)
    {
      --output.count;
    }
  }
  polygon = output;
  return true;
}

inline bool Inside(const Input& input, Point point)
{
  constexpr float TOLERANCE = 2e-7f;
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < -TOLERANCE ||
      point.x > 1.0f + TOLERANCE || point.y < -TOLERANCE || point.y > 1.0f + TOLERANCE)
  {
    return false;
  }
  for (int i = 0; i < input.halfplane_count; ++i)
  {
    const auto& PLANE = input.halfplanes[i];
    const float SCALE =
        1.0f + std::abs(PLANE.x) + std::abs(PLANE.y) + std::abs(PLANE.limit);
    if (PLANE.x * point.x + PLANE.y * point.y - PLANE.limit > TOLERANCE * SCALE)
    {
      return false;
    }
  }
  return true;
}

inline float MotorPower(const Input& input, Point point)
{
  float total = 0.0f;
  for (const auto& MOTOR : input.motors)
  {
    const float CURRENT = MOTOR.u * point.x + MOTOR.v * point.y + MOTOR.c;
    total += MOTOR.k1 * CURRENT * CURRENT + MOTOR.kt * MOTOR.rpm * CURRENT +
             MOTOR.k2 * MOTOR.rpm * MOTOR.rpm;
  }
  return total;
}

inline Quadratic MakeQuadratic(const Input& input)
{
  Quadratic result;
  for (const auto& MOTOR : input.motors)
  {
    const float U = MOTOR.u;
    const float V = MOTOR.v;
    const float C = MOTOR.c;
    const float LINEAR = MOTOR.kt * MOTOR.rpm;
    result.a += MOTOR.k1 * U * U;
    result.b += MOTOR.k1 * U * V;
    result.c += MOTOR.k1 * V * V;
    result.d += (2.0f * MOTOR.k1 * C + LINEAR) * U;
    result.e += (2.0f * MOTOR.k1 * C + LINEAR) * V;
    result.f += MOTOR.k1 * C * C + LINEAR * C + MOTOR.k2 * MOTOR.rpm * MOTOR.rpm;
  }
  return result;
}

struct Search
{
  const Input& input;
  const Quadratic& quadratic;
  Point minimum{};
  float minimum_power = std::numeric_limits<float>::infinity();
  Point best{};
  float best_value = -std::numeric_limits<float>::infinity();
  bool found = false;

  void Minimum(Point point)
  {
    if (!Inside(input, point))
    {
      return;
    }
    const float POWER = MotorPower(input, point);
    if (POWER < minimum_power)
    {
      minimum_power = POWER;
      minimum = point;
    }
  }

  void Candidate(Point point)
  {
    // Validate the actual float coordinates which the firmware will consume.
    point = {std::clamp(point.x, 0.0f, 1.0f), std::clamp(point.y, 0.0f, 1.0f)};
    if (!Inside(input, point))
    {
      return;
    }
    float power = MotorPower(input, point);
    const float POWER_TOLERANCE = 2e-5f + 2e-7f * std::abs(input.power_budget);
    // Rounding an analytic boundary root can put it a few ulps outside. Move
    // toward a known lower-power point, preserving every convex half-plane.
    // Convexity bounds power on this segment without a root-search iteration.
    if (std::isfinite(power) && power > input.power_budget + POWER_TOLERANCE &&
        minimum_power < input.power_budget)
    {
      const float FRACTION = std::min(
          1.0f, (power - input.power_budget + POWER_TOLERANCE) / (power - minimum_power));
      point = {point.x + FRACTION * (minimum.x - point.x),
               point.y + FRACTION * (minimum.y - point.y)};
      power = MotorPower(input, point);
      if (!Inside(input, point))
      {
        return;
      }
    }
    if (!std::isfinite(power) || power > input.power_budget + POWER_TOLERANCE)
    {
      return;
    }
    const float VALUE = input.weights.x * point.x + input.weights.y * point.y;
    if (!found || VALUE > best_value + 2e-7f ||
        (std::abs(VALUE - best_value) <= 2e-7f && point.x + point.y > best.x + best.y))
    {
      best = point;
      best_value = VALUE;
      found = true;
    }
  }

  void Edge(Point start, Point end)
  {
    const Point STEP{end.x - start.x, end.y - start.y};
    const float A = quadratic.a * STEP.x * STEP.x + 2.0f * quadratic.b * STEP.x * STEP.y +
                    quadratic.c * STEP.y * STEP.y;
    const float B = 2.0f * quadratic.a * start.x * STEP.x +
                    2.0f * quadratic.b * (start.x * STEP.y + start.y * STEP.x) +
                    2.0f * quadratic.c * start.y * STEP.y + quadratic.d * STEP.x +
                    quadratic.e * STEP.y;
    const float C = quadratic.Value(start) - input.power_budget;
    if (A > 0.0f)
    {
      const float T = std::clamp(-B / (2.0f * A), 0.0f, 1.0f);
      Minimum({start.x + T * STEP.x, start.y + T * STEP.y});
    }
    if (std::abs(A) <= 2e-7f * (1.0f + std::abs(B) + std::abs(C)))
    {
      if (B != 0.0f)
      {
        const float T = -C / B;
        if (T >= -2e-7f && T <= 1.0f + 2e-7f)
        {
          Candidate({start.x + std::clamp(T, 0.0f, 1.0f) * STEP.x,
                     start.y + std::clamp(T, 0.0f, 1.0f) * STEP.y});
        }
      }
      return;
    }
    float discriminant = B * B - 4.0f * A * C;
    const float DISCRIMINANT_TOLERANCE =
        2e-7f * (B * B + std::abs(4.0f * A * C) + 1e-30f);
    if (discriminant < -DISCRIMINANT_TOLERANCE)
    {
      return;
    }
    discriminant = std::max(0.0f, discriminant);
    const float ROOT = std::sqrt(discriminant);
    // Stable quadratic roots, avoiding cancellation when B and sqrt(delta) are close.
    const float Q = -0.5f * (B + std::copysign(ROOT, B));
    const float ROOTS[2] = {Q / A, Q != 0.0f ? C / Q : -B / (2.0f * A)};
    for (float t : ROOTS)
    {
      if (t >= -2e-7f && t <= 1.0f + 2e-7f)
      {
        t = std::clamp(t, 0.0f, 1.0f);
        Candidate({start.x + t * STEP.x, start.y + t * STEP.y});
      }
    }
  }

  void Interior()
  {
    const float A = quadratic.a;
    const float B = quadratic.b;
    const float C = quadratic.c;
    const float EIGENVALUE = A + C;
    if (EIGENVALUE <= 0.0f)
    {
      return;  // Affine or constant power; polygon edges cover the optimum.
    }
    // Rotate before forming the Gram matrix. Directly subtracting a*c-b*b
    // loses a thin ellipse's small curvature in single precision. Projecting
    // the original motor-current vectors preserves it as a sum of squares.
    const float RATIO = A >= C ? B / A : B / C;
    const float INVERSE_LENGTH = 1.0f / std::sqrt(1.0f + RATIO * RATIO);
    const Point normal = A >= C ? Point{INVERSE_LENGTH, RATIO * INVERSE_LENGTH}
                                : Point{RATIO * INVERSE_LENGTH, INVERSE_LENGTH};
    const Point TANGENT{-normal.y, normal.x};
    float qn = 0.0f;
    float qt = 0.0f;
    float qnt = 0.0f;
    float dn = 0.0f;
    float dt = 0.0f;
    for (const auto& MOTOR : input.motors)
    {
      const float UN = MOTOR.u * normal.x + MOTOR.v * normal.y;
      const float UT = MOTOR.u * TANGENT.x + MOTOR.v * TANGENT.y;
      const float LINEAR = 2.0f * MOTOR.k1 * MOTOR.c + MOTOR.kt * MOTOR.rpm;
      qn += MOTOR.k1 * UN * UN;
      qt += MOTOR.k1 * UT * UT;
      qnt += MOTOR.k1 * UN * UT;
      dn += LINEAR * UN;
      dt += LINEAR * UT;
    }
    const float DN = dn;
    const float DT = dt;
    const float WN = input.weights.x * normal.x + input.weights.y * normal.y;
    const float WT = input.weights.x * TANGENT.x + input.weights.y * TANGENT.y;
    const float DETERMINANT = qn * qt - qnt * qnt;
    if (DETERMINANT > 1e-12f * qn * qn)
    {
      const float CN = (qnt * DT - qt * DN) / (2.0f * DETERMINANT);
      const float CT = (qnt * DN - qn * DT) / (2.0f * DETERMINANT);
      const Point CENTER{normal.x * CN + TANGENT.x * CT, normal.y * CN + TANGENT.y * CT};
      Minimum(CENTER);
      Candidate(CENTER);
      const float RADIUS_SQUARED = input.power_budget - MotorPower(input, CENTER);
      const float DIRECTION_N = (qt * WN - qnt * WT) / DETERMINANT;
      const float DIRECTION_T = (qn * WT - qnt * WN) / DETERMINANT;
      const float NORM = WN * DIRECTION_N + WT * DIRECTION_T;
      if (RADIUS_SQUARED >= 0.0f && NORM > 0.0f)
      {
        const float STEP = std::sqrt(RADIUS_SQUARED / NORM);
        Candidate({CENTER.x + STEP * (normal.x * DIRECTION_N + TANGENT.x * DIRECTION_T),
                   CENTER.y + STEP * (normal.y * DIRECTION_N + TANGENT.y * DIRECTION_T)});
      }
      return;
    }
    // Rank-one PSD: a strip/line, or a parabola when the linear null-space
    // term is nonzero. Edges cover a strip's optimum.
    if (std::abs(DT) > 2e-7f * (1.0f + std::abs(DN) + std::abs(DT)) && WT / DT > 0.0f)
    {
      const float MULTIPLIER = WT / DT;
      const float N = (WN / MULTIPLIER - DN) / (2.0f * EIGENVALUE);
      const float T =
          (input.power_budget - quadratic.f - EIGENVALUE * N * N - DN * N) / DT;
      Candidate({normal.x * N + TANGENT.x * T, normal.y * N + TANGENT.y * T});
    }
  }
};
}  // namespace Detail

inline Result Solve(const Input& input)
{
  Result result;
  if (input.halfplane_count < 0 || input.halfplane_count > 12 ||
      !std::isfinite(input.power_budget) || !std::isfinite(input.weights.x) ||
      !std::isfinite(input.weights.y) || input.weights.x < 0.0f || input.weights.y < 0.0f)
  {
    return result;
  }
  for (const auto& MOTOR : input.motors)
  {
    if (!std::isfinite(MOTOR.u) || !std::isfinite(MOTOR.v) || !std::isfinite(MOTOR.c) ||
        !std::isfinite(MOTOR.rpm) || !std::isfinite(MOTOR.kt) ||
        !std::isfinite(MOTOR.k1) || !std::isfinite(MOTOR.k2) || MOTOR.kt < 0.0f ||
        MOTOR.k1 < 0.0f || MOTOR.k2 < 0.0f)
    {
      return result;
    }
  }
  for (int i = 0; i < input.halfplane_count; ++i)
  {
    const auto& PLANE = input.halfplanes[i];
    if (!std::isfinite(PLANE.x) || !std::isfinite(PLANE.y) || !std::isfinite(PLANE.limit))
    {
      return result;
    }
  }
  // Do not build a polygon or quadratic for the unconstrained common case.
  if (Detail::Inside(input, {1.0f, 1.0f}))
  {
    const float POWER = Detail::MotorPower(input, {1.0f, 1.0f});
    if (std::isfinite(POWER) && POWER <= input.power_budget)
    {
      return {true, true, {1.0f, 1.0f}, POWER, Status::OPTIMAL};
    }
  }

  Detail::Polygon polygon{{{0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}}, 4};
  for (int i = 0; i < input.halfplane_count; ++i)
  {
    if (!Detail::Clip(polygon, input.halfplanes[i]))
    {
      return result;
    }
  }
  result.status = Status::INFEASIBLE;
  if (polygon.count == 0)
  {
    return result;
  }

  // With inactive power constraints a linear objective attains its optimum at
  // a polygon vertex. This also makes hard traction saturation a cheap path.
  Detail::Point vertex = polygon.points[0];
  float vertex_value = input.weights.x * vertex.x + input.weights.y * vertex.y;
  for (int i = 1; i < polygon.count; ++i)
  {
    const auto POINT = polygon.points[i];
    const float VALUE = input.weights.x * POINT.x + input.weights.y * POINT.y;
    if (VALUE > vertex_value ||
        (VALUE == vertex_value && POINT.x + POINT.y > vertex.x + vertex.y))
    {
      vertex = POINT;
      vertex_value = VALUE;
    }
  }
  vertex = {std::clamp(vertex.x, 0.0f, 1.0f), std::clamp(vertex.y, 0.0f, 1.0f)};
  const float VERTEX_POWER = Detail::MotorPower(input, vertex);
  if (Detail::Inside(input, vertex) && std::isfinite(VERTEX_POWER) &&
      VERTEX_POWER <= input.power_budget)
  {
    return {true, true, {vertex.x, vertex.y}, VERTEX_POWER, Status::OPTIMAL};
  }

  const auto QUADRATIC = Detail::MakeQuadratic(input);
  Detail::Search search{input, QUADRATIC};
  for (int i = 0; i < polygon.count; ++i)
  {
    search.Minimum(polygon.points[i]);
    search.Candidate(polygon.points[i]);
    search.Edge(polygon.points[i], polygon.points[(i + 1) % polygon.count]);
  }
  search.Interior();
  search.Candidate(search.minimum);
  result.minimum_available = std::isfinite(search.minimum_power);
  result.feasible = search.found;
  const auto POINT = search.found ? search.best : search.minimum;
  result.point = {POINT.x, POINT.y};
  result.power = Detail::MotorPower(input, {result.point.x, result.point.y});
  result.status = search.found ? Status::OPTIMAL : Status::INFEASIBLE;
  return result;
}
}  // namespace AccelerationConstraint
