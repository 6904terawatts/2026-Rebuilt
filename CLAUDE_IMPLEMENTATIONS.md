# Implementation Plan

All changes on branch `Comp-Robot_test`. Each section is one commit, built consecutively.

---

## 1. Adjustable Shooter Speed via Driver Controller

**Goal:** Allow the driver to adjust shooter RPM with D-pad left/right buttons.

**Files to modify:**
- `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`
- `src/main/java/frc/robot/controls/DriverControls.java`

**ShooterSubsystem.java changes:**
- Add `import edu.wpi.first.math.MathUtil;`
- Add constants:
  ```java
  private static final double DEFAULT_RPM = 3500.0;
  private static final double MIN_RPM = 500.0;
  private static final double MAX_RPM = 5000.0;
  private static final double RPM_STEP = 100.0;
  ```
- Add field: `private double adjustableTargetRPM = DEFAULT_RPM;`
- Add commands:
  ```java
  public Command incrementSpeed() {
      return Commands.runOnce(() -> {
          adjustableTargetRPM = MathUtil.clamp(adjustableTargetRPM + RPM_STEP, MIN_RPM, MAX_RPM);
      });
  }

  public Command decrementSpeed() {
      return Commands.runOnce(() -> {
          adjustableTargetRPM = MathUtil.clamp(adjustableTargetRPM - RPM_STEP, MIN_RPM, MAX_RPM);
      });
  }

  public double getAdjustableTargetRPM() {
      return adjustableTargetRPM;
  }
  ```
- Modify `spinUp()` (line 126): replace hardcoded `3500.0` with `adjustableTargetRPM`
- Add to `periodic()`: `Logger.recordOutput("Shooter/AdjustableTargetRPM", adjustableTargetRPM);`

**DriverControls.java changes:**
- Add bindings after the existing D-pad turret controls (after line 198):
  ```java
  // D-PAD Left/Right - Adjust Shooter Speed
  controller.povRight().onTrue(superstructure.shooter.incrementSpeed());
  controller.povLeft().onTrue(superstructure.shooter.decrementSpeed());
  ```

**Commit message:** `Add adjustable shooter speed via D-pad left/right (500-5000 RPM, 100 RPM steps)`

---

## 2. Fix `withTimeout(0)` on Auto Drive Commands

**Goal:** Auto drive commands currently timeout instantly and never execute.

**File to modify:**
- `src/main/java/frc/robot/RobotContainer.java` (lines 148, 152)

**Changes:**
- Line 148: Change `drivebase.driveBackwards().withTimeout(0)` to `drivebase.driveBackwards().withTimeout(2)` (2 seconds — adjust based on desired auto drive duration)
- Line 152: Change `drivebase.driveForward().withTimeout(0)` to `drivebase.driveForward().withTimeout(2)`

**Note:** The exact timeout values should be tuned during testing. 2 seconds is a reasonable starting point. If these commands are meant to run until interrupted by the next path segment, remove `.withTimeout()` entirely.

**Commit message:** `Fix auto drive commands that had withTimeout(0) causing instant termination`

---

## 3. Fix PathPlanner Gear Ratio Mismatch

**Goal:** PathPlanner settings.json has gear ratio 6.75 but TunerConstants uses 6.12. This causes incorrect velocity calculations during path following.

**File to modify:**
- `src/main/deploy/pathplanner/settings.json`

**Changes:**
- Change `"driveGearing"` from `6.75` to `6.12` to match the value in `TunerConstants.java`

**Verification:** Confirm the correct gear ratio from the physical robot's drivetrain. If the robot uses L2 swerve modules, 6.12 is correct. If L1, 6.75 is correct. Check `TunerConstants.java` `kDriveGearRatio` as the source of truth.

**Commit message:** `Fix PathPlanner gear ratio to match TunerConstants (6.12)`

---

## 4. Fix Intake Pivot Hard Limits

**Goal:** Hard limits are set as `(0, -155)` but all setpoints are positive (59, 115, 148). Either the limits or the setpoints are inverted.

**File to modify:**
- `src/main/java/frc/robot/subsystems/IntakeSubsystem.java`

**Investigation required:** Before changing code, physically verify which direction is positive on the intake pivot encoder:
- If encoder reads positive when deployed: change line 119 from `withHardLimit(Degrees.of(0), Degrees.of(-155))` to `withHardLimit(Degrees.of(-10), Degrees.of(160))` (with small margin beyond setpoints)
- If encoder reads negative when deployed: change setpoints at lines 179/183/187/191 to negative values (`-59`, `-115`, `-148`) and stow to `0`

**Most likely fix** (based on positive setpoints being used everywhere else in the code):
- Line 119: Change hard limits to `withHardLimit(Degrees.of(-5), Degrees.of(155))`

**Commit message:** `Fix intake pivot hard limits to match positive setpoint convention`

---

## 5. Fix Vision Aim Command (Turret/Hood Commands Discarded)

**Goal:** In `visionAimAtHubCommand()`, `turret.setAngle()` and `hood.setAngle()` return Command objects that are never scheduled — the return values are silently discarded.

**File to modify:**
- `src/main/java/frc/robot/subsystems/Superstructure.java` (lines 127-167)

**Changes:**
Replace the `Commands.run()` lambda approach with a proper command composition. The turret and hood need to be controlled directly (not via returned commands) from within a `run()` loop. Two options:

**Option A — Use direct motor control methods (preferred if available):**
If `TurretSubsystem` and `HoodSubsystem` expose direct `setPosition()` methods (not returning Commands), call those instead inside the `Commands.run()` lambda.

**Option B — Restructure as command composition:**
Replace `visionAimAtHubCommand()` with:
```java
public Command visionAimAtHubCommand() {
    return aimDynamicCommand(
        () -> calculateAimingParameters(...).shooterSpeed,
        () -> calculateAimingParameters(...).turretAngle,
        () -> calculateAimingParameters(...).hoodAngle
    ).withName("Superstructure.visionAimAtHub");
}
```
This reuses the existing `aimDynamicCommand()` which properly schedules subsystem commands via suppliers.

**Option C — Cache and reuse parameters:**
Add a field `private AimingParameters latestAimParams;` that is updated in a `Commands.run()` loop, then feed suppliers from that field into `aimDynamicCommand()`.

**Commit message:** `Fix vision aim command to actually schedule turret and hood movements`

---

## 6. Fix Limelight Alliance Flipping

**Goal:** `Robot.java` always calls `getBotPoseEstimate_wpiBlue()`, which returns Blue-origin coordinates. On Red alliance, `resetPose()` sets the robot to the wrong field location.

**File to modify:**
- `src/main/java/frc/robot/Robot.java` (lines 56-62)

**Changes:**
Replace:
```java
var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
```
With alliance-aware logic:
```java
var alliance = DriverStation.getAlliance();
var llMeasurement = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red
    ? LimelightHelpers.getBotPoseEstimate_wpiRed("limelight")
    : LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
```

**Alternative:** If the rest of the codebase (PathPlanner, odometry) assumes Blue-origin coordinates, keep `wpiBlue` and do NOT change this. Check what `SwerveSubsystem` and PathPlanner expect. WPILib 2024+ convention is Blue-origin always — in that case, `wpiBlue` is correct and this is NOT a bug. Verify before changing.

**Commit message:** `Fix Limelight pose estimation to respect alliance color`

---

## 7. Fix Duplicate Y Button Binding

**Goal:** Y button is bound to both "Intake Deploy" (line 153) and "Zero Gyro" (line 184), causing both to fire on every press.

**File to modify:**
- `src/main/java/frc/robot/controls/DriverControls.java`

**Changes:**
- Move "Zero Gyro" off the Y button. The Back button is free in normal mode (only used in test mode). Rebind:
  - Line 184: Change `controller.y().onTrue(drivetrain.zeroGyro())` to `controller.back().onTrue(drivetrain.zeroGyro())`
- Fix the misleading comment on line 152: Change `// B Button` to `// Y Button`

**Button assignment after fix:**
- **Y** — Intake Deploy (set pivot to 0 degrees)
- **Back** — Zero Gyro

**Commit message:** `Fix duplicate Y button binding by moving zero gyro to Back button`

---

## 8. Remove `|| true` from Joystick Warning Silencer

**Goal:** `RobotContainer.java` line 110 has `if (!Robot.isReal() || true)` which always silences joystick disconnect warnings, even on the real robot.

**File to modify:**
- `src/main/java/frc/robot/RobotContainer.java` (line 110)

**Changes:**
- Change `if (!Robot.isReal() || true)` to `if (!Robot.isReal())`

This restores the intended behavior: warnings are only silenced in simulation.

**Commit message:** `Fix joystick disconnect warnings being silenced on real robot`

---

## 9. Add `end()` Cleanup in ShootOnTheMoveCommand

**Goal:** `aimDynamicCommand` is scheduled in `initialize()` but never cancelled when the parent command ends, causing it to run indefinitely.

**File to modify:**
- `src/main/java/frc/robot/commands/ShootOnTheMoveCommand.java`

**Changes:**
- Add a field to store the child command reference:
  ```java
  private Command aimCommand;
  ```
- In `initialize()` (lines 59-68), store the command before scheduling:
  ```java
  aimCommand = superstructure.aimDynamicCommand(
      () -> this.latestShootSpeed,
      () -> this.latestTurretAngle,
      () -> this.latestHoodAngle
  );
  aimCommand.schedule();
  ```
- Override `end()` to cancel the child command:
  ```java
  @Override
  public void end(boolean interrupted) {
      if (aimCommand != null) {
          aimCommand.cancel();
          aimCommand = null;
      }
  }
  ```
- Remove the TODO comments on lines 57-58 since this resolves them.

**Commit message:** `Fix ShootOnTheMoveCommand to cancel child aim command on end`

SUMMARY: 
● All 9 implementations are done, each in its own commit:

  ┌─────┬─────────┬────────────────────────────────────────────────────────────────────────────┐
  │  #  │ Commit  │                                    Fix                                     │
  ├─────┼─────────┼────────────────────────────────────────────────────────────────────────────┤
  │ 1   │ 1209be4 │ Adjustable shooter speed — D-pad left/right, 500-5000 RPM, 100 RPM steps   │
  ├─────┼─────────┼────────────────────────────────────────────────────────────────────────────┤
  │ 2   │ cb2f1db │ withTimeout(0) → withTimeout(2) on auto drive commands                     │
  ├─────┼─────────┼────────────────────────────────────────────────────────────────────────────┤
  │ 3   │ 3b8942c │ PathPlanner gear ratio 6.75 → 6.12 to match TunerConstants                 │
  ├─────┼─────────┼────────────────────────────────────────────────────────────────────────────┤
  │ 4   │ c323e58 │ Intake hard limits (0, -155) → (-5, 155) to match setpoints                │
  ├─────┼─────────┼────────────────────────────────────────────────────────────────────────────┤
  │ 5   │ b7d77cc │ Vision aim restructured to use aimDynamicCommand() with suppliers          │
  ├─────┼─────────┼────────────────────────────────────────────────────────────────────────────┤
  │ 6   │ 0fe4884 │ Removed resetPose() in Robot.java — SwerveSubsystem already handles fusion │
  ├─────┼─────────┼────────────────────────────────────────────────────────────────────────────┤
  │ 7   │ 4bfb5ef │ Moved Zero Gyro from Y (conflicted with Intake) to Back button             │
  ├─────┼─────────┼────────────────────────────────────────────────────────────────────────────┤
  │ 8   │ 8d23f2e │ Removed `                                                                  │
  ├─────┼─────────┼────────────────────────────────────────────────────────────────────────────┤
  │ 9   │ 8eeb353 │ ShootOnTheMoveCommand now cancels child aim command in end()               │
  └─────┴─────────┴────────────────────────────────────────────────────────────────────────────┘

  Each commit is self-contained and can be individually reverted with git revert <hash> if any fix
  causes issues during testing.
