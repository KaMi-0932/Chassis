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
- The left switch selects RELAX / FOLLOW / INDEPENDENT from top to bottom. The
  right switch selects gimbal RELAX / COMMON / COMMON, matching the source.
- Existing hardware registrations, CAN receive/transport implementation, other
  module constructor interfaces, and configurable Topic names are retained.

## Drive-only power policy

Only the four M3508 drive motors enter the model estimate and Helm power budget.
The default product sets `motor_count_6020` to zero. The shared Helm limiter also
passes steering requests through for configurations that still register 6020s;
steering retains its local PID and +/-2.5 N*m output clamp.

The drive budget is the referee/boost limit minus the 4 W control margin. Steering
and static board losses are not charged against this budget. RLS uses the measured
power minus configured static loss to identify drive losses; this assumes that
the measured power supplied by SuperPower covers the drive branch. Check the
actual power-meter wiring before relying on online identification.

Raw RMMotor feedback torque is converted to the direction used by feedback RPM
before power estimation and RLS. The limiter keeps current within the original
request magnitude and sign. When a budget is physically infeasible under the
model, it chooses the minimum-power permitted currents rather than claiming the
budget can always be met. Telemetry powers are model estimates, not measurements;
the steering estimate remains zero.

`moment_of_inertia` defaults to zero in the other product configurations. It is
forwarded only to Helm; Omni and Mecanum keep their current parameter contracts.
Module edits live in the modules' own Git repositories and are not captured by
the parent BSP's lock file until the module commits are published and relocked.

## Hardware validation still required

1. With no projectiles and the wheels unloaded, check CAN IDs, wheel ordering,
   reverse flags, steering zeros, reduction ratio and feedback units. Confirm
   RELAX and loss of remote control remove motor torque.
2. Apply small +y, +x and pure rotation commands. Check wheel angle, drive sign,
   and measured chassis velocity; verify FOLLOW decreases the yaw error.
3. Check steering hold at zero input and reduced wheel targets during steering
   alignment. Record estimated drive power, actual metered power and limited
   current without including steering in the requested budget.
4. Only after those checks, test on the floor at low speed. Observe braking,
   mode transitions, feedback validity, current and power before raising limits.

Compilation and host power-limiter tests do not validate physical direction,
power-meter coverage, PID tuning, or closed-loop stability on the robot.
