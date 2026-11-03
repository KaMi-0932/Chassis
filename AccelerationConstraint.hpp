#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

/**
 * Two-dimensional acceleration allocation, following the document's polygon,
 * ellipse-support and edge-intersection construction. Motor speeds and wheel PID
 * outputs are frozen in Input. Powers are signed; braking power is not clipped.
 * Decision coordinates are normalized acceleration fractions in [0, 1].
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
  double x;
  double y;
};

struct Polygon
{
  Point points[20]{};
  int count = 0;
};

struct Quadratic
{
  double a = 0.0;
  double b = 0.0;  // f = a*x*x + 2*b*x*y + c*y*y + d*x + e*y + f.
  double c = 0.0;
  double d = 0.0;
  double e = 0.0;
  double f = 0.0;

  double Value(Point p) const
  {
    return (a * p.x + 2.0 * b * p.y + d) * p.x + (c * p.y + e) * p.y + f;
  }
};

inline bool AddPoint(Polygon& polygon, Point point)
{
  if (polygon.count > 0)
  {
    const Point LAST = polygon.points[polygon.count - 1];
    if (std::abs(point.x - LAST.x) + std::abs(point.y - LAST.y) < 1e-12)
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
  double previous_distance = plane.x * previous.x + plane.y * previous.y - plane.limit;
  for (int i = 0; i < polygon.count; ++i)
  {
    const Point CURRENT = polygon.points[i];
    const double DISTANCE = plane.x * CURRENT.x + plane.y * CURRENT.y - plane.limit;
    if ((DISTANCE <= 0.0) != (previous_distance <= 0.0))
    {
      const double T = previous_distance / (previous_distance - DISTANCE);
      if (!AddPoint(output, {previous.x + T * (CURRENT.x - previous.x),
                             previous.y + T * (CURRENT.y - previous.y)}))
      {
        return false;
      }
    }
    if (DISTANCE <= 0.0 && !AddPoint(output, CURRENT))
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
    if (std::abs(FIRST.x - LAST.x) + std::abs(FIRST.y - LAST.y) < 1e-12)
    {
      --output.count;
    }
  }
  polygon = output;
  return true;
}

inline bool Inside(const Input& input, Point point)
{
  constexpr double TOLERANCE = 2e-7;
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < -TOLERANCE ||
      point.x > 1.0 + TOLERANCE || point.y < -TOLERANCE || point.y > 1.0 + TOLERANCE)
  {
    return false;
  }
  for (int i = 0; i < input.halfplane_count; ++i)
  {
    const auto& PLANE = input.halfplanes[i];
    const double SCALE =
        1.0 + std::abs(PLANE.x) + std::abs(PLANE.y) + std::abs(PLANE.limit);
    if (PLANE.x * point.x + PLANE.y * point.y - PLANE.limit > TOLERANCE * SCALE)
    {
      return false;
    }
  }
  return true;
}

inline double MotorPower(const Input& input, Point point)
{
  double total = 0.0;
  for (const auto& MOTOR : input.motors)
  {
    const double CURRENT = MOTOR.u * point.x + MOTOR.v * point.y + MOTOR.c;
    total += MOTOR.k1 * CURRENT * CURRENT +
             static_cast<double>(MOTOR.kt) * MOTOR.rpm * CURRENT +
             static_cast<double>(MOTOR.k2) * MOTOR.rpm * MOTOR.rpm;
  }
  return total;
}

inline Quadratic MakeQuadratic(const Input& input)
{
  Quadratic result;
  for (const auto& MOTOR : input.motors)
  {
    const double U = MOTOR.u;
    const double V = MOTOR.v;
    const double C = MOTOR.c;
    const double LINEAR = static_cast<double>(MOTOR.kt) * MOTOR.rpm;
    result.a += MOTOR.k1 * U * U;
    result.b += MOTOR.k1 * U * V;
    result.c += MOTOR.k1 * V * V;
    result.d += (2.0 * MOTOR.k1 * C + LINEAR) * U;
    result.e += (2.0 * MOTOR.k1 * C + LINEAR) * V;
    result.f += MOTOR.k1 * C * C + LINEAR * C +
                static_cast<double>(MOTOR.k2) * MOTOR.rpm * MOTOR.rpm;
  }
  return result;
}

struct Search
{
  const Input& input;
  const Quadratic& quadratic;
  Point minimum{};
  double minimum_power = std::numeric_limits<double>::infinity();
  Point best{};
  double best_value = -std::numeric_limits<double>::infinity();
  bool found = false;

  void Minimum(Point point)
  {
    if (!Inside(input, point))
    {
      return;
    }
    const double POWER = MotorPower(input, point);
    if (POWER < minimum_power)
    {
      minimum_power = POWER;
      minimum = point;
    }
  }

  void Candidate(Point point)
  {
    // Validate the actual float coordinates which the firmware will consume.
    point = {static_cast<float>(std::clamp(point.x, 0.0, 1.0)),
             static_cast<float>(std::clamp(point.y, 0.0, 1.0))};
    if (!Inside(input, point))
    {
      return;
    }
    const double POWER = MotorPower(input, point);
    const double POWER_TOLERANCE = 2e-5 + 2e-7 * std::abs(input.power_budget);
    if (!std::isfinite(POWER) || POWER > input.power_budget + POWER_TOLERANCE)
    {
      return;
    }
    const double VALUE = input.weights.x * point.x + input.weights.y * point.y;
    if (!found || VALUE > best_value + 1e-12 ||
        (std::abs(VALUE - best_value) <= 1e-12 && point.x + point.y > best.x + best.y))
    {
      best = point;
      best_value = VALUE;
      found = true;
    }
  }

  void Edge(Point start, Point end)
  {
    const Point STEP{end.x - start.x, end.y - start.y};
    const double A = quadratic.a * STEP.x * STEP.x + 2.0 * quadratic.b * STEP.x * STEP.y +
                     quadratic.c * STEP.y * STEP.y;
    const double B = 2.0 * quadratic.a * start.x * STEP.x +
                     2.0 * quadratic.b * (start.x * STEP.y + start.y * STEP.x) +
                     2.0 * quadratic.c * start.y * STEP.y + quadratic.d * STEP.x +
                     quadratic.e * STEP.y;
    const double C = quadratic.Value(start) - input.power_budget;
    if (A > 0.0)
    {
      const double T = std::clamp(-B / (2.0 * A), 0.0, 1.0);
      Minimum({start.x + T * STEP.x, start.y + T * STEP.y});
    }
    if (std::abs(A) <= 1e-14 * (1.0 + std::abs(B) + std::abs(C)))
    {
      if (B != 0.0)
      {
        const double T = -C / B;
        if (T >= -1e-10 && T <= 1.0 + 1e-10)
        {
          Candidate({start.x + std::clamp(T, 0.0, 1.0) * STEP.x,
                     start.y + std::clamp(T, 0.0, 1.0) * STEP.y});
        }
      }
      return;
    }
    double discriminant = B * B - 4.0 * A * C;
    const double DISCRIMINANT_TOLERANCE = 1e-12 * (B * B + std::abs(4.0 * A * C) + 1e-30);
    if (discriminant < -DISCRIMINANT_TOLERANCE)
    {
      return;
    }
    discriminant = std::max(0.0, discriminant);
    const double ROOT = std::sqrt(discriminant);
    // Stable quadratic roots, avoiding cancellation when B and sqrt(delta) are close.
    const double Q = -0.5 * (B + std::copysign(ROOT, B));
    const double ROOTS[2] = {Q / A, Q != 0.0 ? C / Q : -B / (2.0 * A)};
    for (double t : ROOTS)
    {
      if (t >= -1e-10 && t <= 1.0 + 1e-10)
      {
        t = std::clamp(t, 0.0, 1.0);
        Candidate({start.x + t * STEP.x, start.y + t * STEP.y});
      }
    }
  }

  void Interior()
  {
    const double A = quadratic.a;
    const double B = quadratic.b;
    const double C = quadratic.c;
    const double D = quadratic.d;
    const double E = quadratic.e;
    const double DETERMINANT = A * C - B * B;
    const double SCALE = std::max(A, C);
    if (DETERMINANT > 1e-14 * SCALE * SCALE && SCALE > 0.0)
    {
      const Point CENTER{(B * E - C * D) / (2.0 * DETERMINANT),
                         (B * D - A * E) / (2.0 * DETERMINANT)};
      Minimum(CENTER);
      Candidate(CENTER);
      const double RADIUS_SQUARED = input.power_budget - quadratic.Value(CENTER);
      const Point DIRECTION{(C * input.weights.x - B * input.weights.y) / DETERMINANT,
                            (A * input.weights.y - B * input.weights.x) / DETERMINANT};
      const double NORM = input.weights.x * DIRECTION.x + input.weights.y * DIRECTION.y;
      if (RADIUS_SQUARED >= 0.0 && NORM > 0.0)
      {
        const double STEP = std::sqrt(RADIUS_SQUARED / NORM);
        Candidate({CENTER.x + STEP * DIRECTION.x, CENTER.y + STEP * DIRECTION.y});
      }
      return;
    }
    const double EIGENVALUE = A + C;
    if (EIGENVALUE <= 0.0)
    {
      return;  // Affine or constant power; polygon edges cover the optimum.
    }
    // Rank-one PSD: a strip/line, or a parabola when the linear null-space term is
    // nonzero.
    Point normal = A >= C ? Point{A, B} : Point{B, C};
    const double LENGTH = std::hypot(normal.x, normal.y);
    normal.x /= LENGTH;
    normal.y /= LENGTH;
    const Point TANGENT{-normal.y, normal.x};
    const double DN = D * normal.x + E * normal.y;
    const double DT = D * TANGENT.x + E * TANGENT.y;
    const double WN = input.weights.x * normal.x + input.weights.y * normal.y;
    const double WT = input.weights.x * TANGENT.x + input.weights.y * TANGENT.y;
    if (std::abs(DT) > 1e-12 * (1.0 + std::abs(D) + std::abs(E)) && WT / DT > 0.0)
    {
      const double MULTIPLIER = WT / DT;
      const double N = (WN / MULTIPLIER - DN) / (2.0 * EIGENVALUE);
      const double T =
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
  Detail::Polygon polygon{{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}}, 4};
  for (int i = 0; i < input.halfplane_count; ++i)
  {
    const auto& PLANE = input.halfplanes[i];
    if (!std::isfinite(PLANE.x) || !std::isfinite(PLANE.y) ||
        !std::isfinite(PLANE.limit) || !Detail::Clip(polygon, PLANE))
    {
      return result;
    }
  }
  result.status = Status::INFEASIBLE;
  if (polygon.count == 0)
  {
    return result;
  }
  const auto QUADRATIC = Detail::MakeQuadratic(input);
  Detail::Search search{input, QUADRATIC};
  // Monotone objective: the fully requested acceleration is globally optimal if feasible.
  if (Detail::Inside(input, {1.0, 1.0}) &&
      Detail::MotorPower(input, {1.0, 1.0}) <= input.power_budget)
  {
    result.feasible = true;
    result.minimum_available = true;
    result.point = {1.0f, 1.0f};
    result.power = static_cast<float>(Detail::MotorPower(input, {1.0, 1.0}));
    result.status = Status::OPTIMAL;
    return result;
  }
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
  result.point = {static_cast<float>(POINT.x), static_cast<float>(POINT.y)};
  result.power =
      static_cast<float>(Detail::MotorPower(input, {result.point.x, result.point.y}));
  result.status = search.found ? Status::OPTIMAL : Status::INFEASIBLE;
  return result;
}
}  // namespace AccelerationConstraint
