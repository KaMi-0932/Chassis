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
- The current default configuration selects RELAX / FOLLOW / ROTOR from top to
  bottom on the left switch. The right switch selects gimbal RELAX / COMMON / COMMON.
- Existing hardware registrations, CAN receive/transport implementation, other
  module constructor interfaces, and configurable Topic names are retained.

## Upstream power policy

PowerControl is restored to QDU-Robomaster/PowerControl at upstream commit
`4097e1f6a3fad89126e82b920217961286966d84`. The Helm power-data submission, RLS
invocation and referee/boost budget selection follow
QDU-Robomaster/Chassis commit `32761d015294031f3d5ac8326b1a394c796302a9`.

Only the four M3508 drive motors enter power limiting. The Helm configurations
set `motor_count_6020` to zero, and Helm submits only M3508 data to PowerControl.
Limited output currents replace only drive-wheel torque requests; steering keeps
its PID output and local +/-2.5 N*m torque clamp. The upstream limiter sums positive
drive model powers, with static chassis loss and the 4 W margin deducted from the
budget. This drive-only policy is the intentional exception to upstream Helm
power integration; the PowerControl module itself is unmodified.

Additional Helm power estimates, Ozone sampling globals, and the velocity debug
pointer are removed. Physical chassis-velocity feedback remains part of the
migrated motion controller. The motor target names at the power-control boundary
match upstream: `target_speed_` is motor rotor RPM and `target_angle_` is radians.
RMMotor decodes torque using the same configured direction as velocity, so power
identification consumes aligned feedback directly; the extra torque-polarity
interface is no longer needed.
Confirm the SuperPower protocol and actual power-meter coverage before relying
on online power identification.

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
   alignment. Check actual metered drive power and limited drive currents against
   the drive budget. Steering power is outside that budget; verify the measurement
   branch before using online identification.
4. Only after those checks, test on the floor at low speed. Observe braking,
   mode transitions, feedback validity, current and power before raising limits.

Compilation does not validate physical direction,
power-meter coverage, PID tuning, or closed-loop stability on the robot.
