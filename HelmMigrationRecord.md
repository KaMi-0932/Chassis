# Helm migration from luyan2

The source is the working tree of `D:/Documents/luyan2`, including its local module
changes. `User/xrobot.yaml` is converted to the current XRobot constructor/YAML
contracts. The older `User/RobotConfig/helm_infantry.yaml` in the source contains
only BlinkLED and is not the product configuration used for this migration.

## Control and configuration

- The control thread runs every 2 ms. Commands are normalized inputs; the Helm
  controller converts them to m/s and rad/s using wheel radius, reduction ratio,
  the 9000 RPM motor limit, and wheel-to-center distance.
- Coordinates are +x right, +y forward, and positive angular velocity
  counterclockwise. Wheel indices are 0 left rear, 1 right rear, 2 right front,
  3 left front. Geometry assumes four symmetric 45-degree wheel positions.
- Steering zero is along +y, with positive steering angle toward +x. The existing
  four zero offsets are retained. The wheel radius is 0.06 m, wheel-to-center
  distance 0.233 m, reduction ratio 14, chassis weight 182.692 N, and yaw inertia
  0.49436791247 kg*m^2, matching the source product.
- The source forward/inverse kinematics, zero-speed steering hold, optimal-angle
  reversal, cos-cubed speed compensation, chassis-velocity feedback, and wheel
  torque feedforward are retained. Wheel-speed PID uses observed chassis velocity
  projected along each actual steering direction, matching the source.
- INDEPENDENT and ROTOR use the joystick z command for target angular velocity.
  FOLLOW uses a 2-degree deadband on the signed shortest yaw error: inside the
  deadband, target angular velocity is zero and the follow PID is reset. Outside
  it, normal following resumes. Kinematics and inverse dynamics share that target;
  the deadband has no hysteresis.
- The current default configuration selects RELAX / FOLLOW / ROTOR from top to
  bottom on the left switch. The right switch selects gimbal RELAX / COMMON / COMMON.
- Existing hardware registrations, CAN receive/transport implementation, other
  module constructor interfaces, and configurable Topic names are retained.

## Acceleration, traction and signed power constraints

`AccelerationConstraint.hpp` follows sections 1.1.5 and 1.1.6 of the supplied
swerve-controller report: Sutherland-Hodgman half-plane clipping, feasible polygon
vertices, quadratic/segment intersections, and the power ellipse's objective
support point. Rank-zero/rank-one quadratics, point/line polygons, negative budgets,
and infeasible requests are handled without changing the physical quadratic.
The solver uses fixed arrays and no dynamic allocation. Its arithmetic is single
precision for the M4F hardware FPU. It first checks the full request, then the
polygon's objective-maximizing vertex, before enumerating power intersections and
support points. Thin ellipses use a rotated Gram matrix constructed from motor
current coefficients. Rounded boundary candidates are checked against the actual
float current/power expressions and, if needed, moved toward a known lower-power
feasible point without relaxing the power limit.

Decision coordinates are retained fractions x,y in [0,1] of requested translation
and angular acceleration. Translation direction and angular acceleration sign are
preserved. The objective is translation_weight*x + angular_weight*y. Defaults are
1:1 in normal modes and 1:4 in ROTOR, a weighted preference rather than strict
lexicographic priority. YAML fields `translation_acceleration_weight`,
`angular_acceleration_weight` and `rotor_angular_acceleration_weight` are forwarded
only to Helm. Zero requested axes have zero objective weight.

The document's equal-load traction diamond is
|a|/(mu*g) + |alpha|*J/(mu*weight*R) <= 1. The user measured mu approximately 0.4;
the requested conservative configured `friction_coefficient` is 0.35. Extra affine
half-planes bound each actual PID+FF wheel torque by the minimum of 4.4 N*m,
the 16384-LSB hardware limit, and mu*weight*wheel_radius/4. Thus the PID offset
cannot bypass the longitudinal drive traction bound. Normal load is assumed equal;
load transfer, lateral tire force and transient tire dynamics are not modeled.
The constraints are model-based and do not guarantee absence of real wheel slip.

Motor speeds and wheel PID outputs are frozen once per cycle. Per-wheel current
is affine in x,y: I_i=u_i*x+v_i*y+c_i. Power is the SIGNED sum of
kt*I_i*rpm_i + k1*I_i^2 + k2*rpm_i^2. Regenerative negative power is not clipped or
counted twice. The constant PID loss term is k1*c_i^2. RMMotor already aligns
feedback torque with feedback RPM, so no extra polarity interface is used.

PowerControl retains its existing RLS/legacy limiters, adding only a locked
GetDrivePowerModel() coefficient snapshot. Helm no longer invokes the legacy
positive-power limiter or replaces outputs with its result. Only four drive motors
are constrained; motor_count_6020 remains zero and steering keeps its PID output
and +/-2.5 N*m clamp. Static chassis loss and the existing 4 W control margin are
deducted from the available chassis budget before solving.

Referee data older than one second, or a missing/invalid limit, selects
HELM_CHASSIS_MAX_POWER. Existing supercap policy is preserved: only online BOOST
adds 100/70/40 W when feedback exceeds 0.8/0.5/0.25 respectively. GetCapEnergy()
currently decodes an output-capability ratio, not a separately measured energy
in joules; this board-protocol interpretation still needs hardware confirmation.

If full frozen PID makes the constrained request infeasible, solve again at PID
scales 0.5 and 0 without advancing PID state. If the solver still reports
INFEASIBLE, drive torque is zero while valid steering control continues, avoiding
relax/re-enable cycles of the steering motors. Invalid feedback, commands, timing,
model or solver input still relax all motors, as does RELAX. Zero drive current
is an explicit fallback, not a claim that impossible modeled power can be met:
the speed-loss term alone can exceed the budget. `acceleration_constraint_result_`,
`constraint_pid_scale_`, `constraint_output_valid_` and `constraint_infeasible_`
retain the result/status inside Helm for diagnosis.

The control thread uses a 2 ms scheduled period. A missed release is discarded
and the schedule is restarted, avoiding back-to-back catch-up iterations. This
does not make an overlong computation meet its deadline. `Sleep(1)` would instead
add a tick delay to variable computation time and does not enforce a 2 ms period.

`control_diagnostics_` is an instance member, with no fixed RAM globals or printing
in the control loop. Work durations use the existing microsecond timebase and
include interrupts, preemption and mutex waits. Inspect these on the MCU:

| Member | Meaning |
| --- | --- |
| `period_us`, `max_period_us` | Update-to-Update interval, including sleep, matching `dt_` |
| `loop_time_us`, `max_loop_time_us` | Complete iteration work, excluding scheduled sleep |
| `loop_overrun_count` | Iterations with work lasting at least 2000 us |
| `period_over_3ms_count` | Update intervals lasting at least 3000 us |
| `power_control_time_us`, `max_power_control_time_us` | Entire power/traction update, including all retries |
| `model_update_time_us`, `max_model_update_time_us` | Motor-data submission, online identification and model snapshot |
| `solve_time_us`, `max_solve_time_us` | Last individual solve and maximum individual solve |
| `total_solve_time_us`, `solver_attempts` | Sum of solve times and attempt count in the latest iteration |
| `output_time_us`, `max_output_time_us` | Motor command submission, including any driver logging |
| `infeasible_count`, `invalid_solver_input_count` | Failed solver cycles, excluding early feedback/model rejection |
| `stack_min_free_bytes` | FreeRTOS stack high-water mark in bytes, sampled every 256 iterations when enabled |
| `drive_power_budget`, `power_model` | Budget after static loss/margin, and live coefficients |
| `requested_ax`, `requested_ay`, `requested_alpha` | Acceleration requests before constraint scaling |

Watch maxima over repeated constraint activation and all three retry paths;
average solve duration cannot establish a 2 ms worst-case bound. If solver time
is short while iteration time is long, investigate other tasks, driver output
and logging. This driver can log on every command when a motor exceeds 75 C.

Other product configurations retain their constructor contract after xrobot sync;
only Helm receives the new fields. No vendor code is changed. Module changes remain
local to their own repositories until explicitly committed, published and relocked.

## Hardware validation still required

1. With no projectiles and the wheels unloaded, check CAN IDs, wheel ordering,
   reverse flags, steering zeros, reduction ratio and feedback units. Confirm
   RELAX and loss of remote control remove motor torque.
2. Apply small +y, +x and pure rotation commands. Check wheel angle, drive sign,
   and measured chassis velocity; verify FOLLOW decreases the yaw error.
3. Check steering hold at zero input and reduced wheel targets during steering
   alignment. Check actual metered drive power and limited drive currents against
   the drive budget. Steering power is outside that budget; verify the measurement
   branch before using online identification.
4. Only after those checks, test on the floor at low speed. Observe braking,
   mode transitions, feedback validity, current and power before raising limits.

Before enabling higher limits, measure DWT cycle count and task stack high-water
mark on the MCU, including the worst-case three solver attempts, against the
2 ms control period. Compilation and host solver tests do not establish timing.

Compilation does not validate physical direction,
power-meter coverage, PID tuning, or closed-loop stability on the robot.
