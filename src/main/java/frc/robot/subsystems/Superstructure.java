package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/**
 * Superstructure coordinates the shooter, turret, hood, and intake subsystems
 * for unified control during shooting operations.
 * 
 * NEW: Vision-Based Aiming!
 * The superstructure can now use the VisionSubsystem to automatically aim at the hub
 * by calculating the correct turret and hood angles based on the hub's actual position.
 */
public class Superstructure extends SubsystemBase {

  public final ShooterSubsystem shooter;
  public final TurretSubsystem turret;
  public final HoodSubsystem hood;
  public final IntakeSubsystem intake;
  public final HopperSubsystem hopper;
  public final KickerSubsystem kicker;
  
  // Vision subsystem for vision-based aiming (Optional because it might not be available)
  private Optional<VisionSubsystem> visionSubsystem = Optional.empty();
  
  // Swerve subsystem for getting robot pose (needed for aiming calculations)
  private Optional<SwerveSubsystem> swerveSubsystem = Optional.empty();

  // Tolerance for "at setpoint" checks
  private static final AngularVelocity SHOOTER_TOLERANCE = RPM.of(100);
  private static final Angle TURRET_TOLERANCE = Degrees.of(1);
  private static final Angle HOOD_TOLERANCE = Degrees.of(2);

  // Triggers for readiness checks
  private final Trigger isShooterAtSpeed;
  private final Trigger isTurretOnTarget;
  private final Trigger isHoodOnTarget;
  private final Trigger isReadyToShoot;

  private AngularVelocity targetShooterSpeed = RPM.of(0);
  private Angle targetTurretAngle = Degrees.of(0);
  private Angle targetHoodAngle = Degrees.of(0);

  // Default aim point is red hub
  private Translation3d aimPoint = Constants.AimPoints.RED_HUB.value;

  public Superstructure(ShooterSubsystem shooter, TurretSubsystem turret, HoodSubsystem hood, IntakeSubsystem intake,
      HopperSubsystem hopper, KickerSubsystem kicker) {
    this.shooter = shooter;
    this.turret = turret;
    this.hood = hood;
    this.intake = intake;
    this.hopper = hopper;
    this.kicker = kicker;

    // Create triggers for checking if mechanisms are at their targets
    this.isShooterAtSpeed = new Trigger(
        () -> Math.abs(shooter.getSpeed().in(RPM) - targetShooterSpeed.in(RPM)) < SHOOTER_TOLERANCE.in(RPM));

    this.isTurretOnTarget = new Trigger(
        () -> Math.abs(turret.getRawAngle().in(Degrees) - targetTurretAngle.in(Degrees)) < TURRET_TOLERANCE
            .in(Degrees));

    this.isHoodOnTarget = new Trigger(
        () -> Math.abs(hood.getAngle().in(Degrees) - targetHoodAngle.in(Degrees)) < HOOD_TOLERANCE.in(Degrees));

    this.isReadyToShoot = isShooterAtSpeed.and(isTurretOnTarget).and(isHoodOnTarget);
  }
  
  /**
   * Sets the vision subsystem for vision-based aiming.
   * 
   * CALL THIS FROM ROBOTCONTAINER after creating the VisionSubsystem!
   * 
   * @param vision The VisionSubsystem to use
   */
  public void setVisionSubsystem(VisionSubsystem vision) {
    this.visionSubsystem = Optional.of(vision);
  }
  
  /**
   * Sets the swerve subsystem for robot pose information.
   * 
   * NEEDED FOR: Calculating aim angles based on robot position
   * 
   * @param swerve The SwerveSubsystem to use
   */
  public void setSwerveSubsystem(SwerveSubsystem swerve) {
    this.swerveSubsystem = Optional.of(swerve);
  }
  
  /**
   * Automatically aims at the hub using vision and odometry.
   * 
   * HOW IT WORKS:
   * 1. Get the hub's position (from vision if available, otherwise use known position)
   * 2. Get our robot's current position and orientation
   * 3. Calculate the angle and distance to the hub
   * 4. Set turret and hood to the correct angles
   * 5. Adjust shooter speed based on distance
   * 
   * This command continuously updates - it's "closed-loop" aiming!
   * 
   * @return Command that aims at the hub and maintains aim
   */
  public Command visionAimAtHubCommand() {
    return Commands.parallel(
        // Continuously calculate aiming parameters and update setpoints
        Commands.run(() -> {
          // Get current alliance to know which hub to aim at
          boolean isRed = DriverStation.getAlliance()
              .map(alliance -> alliance == DriverStation.Alliance.Red)
              .orElse(false);

          // Get the hub position (vision will try to use AprilTags first)
          Translation3d hubPosition = visionSubsystem
              .map(vision -> vision.getHubPosition(isRed))
              .orElse(isRed ? Constants.AimPoints.RED_HUB.value : Constants.AimPoints.BLUE_HUB.value);

          // Get our robot's current pose
          Translation2d robotPosition = swerveSubsystem
              .map(swerve -> swerve.getPose().getTranslation())
              .orElse(new Translation2d());

          Rotation2d robotHeading = swerveSubsystem
              .map(swerve -> swerve.getPose().getRotation())
              .orElse(new Rotation2d());

          // Calculate aiming parameters
          AimingParameters params = calculateAimingParameters(
              robotPosition,
              robotHeading,
              hubPosition
          );

          // Store setpoints so aimDynamicCommand suppliers can read them
          setShooterSetpoints(params.shooterSpeed, params.turretAngle, params.hoodAngle);

          // Log the aiming data
          Logger.recordOutput("Superstructure/VisionAiming/TargetHub", hubPosition);
          Logger.recordOutput("Superstructure/VisionAiming/Distance", params.distanceToHub);
          Logger.recordOutput("Superstructure/VisionAiming/TurretAngle", params.turretAngle.in(Degrees));
          Logger.recordOutput("Superstructure/VisionAiming/HoodAngle", params.hoodAngle.in(Degrees));

        }, this),
        // Drive mechanisms to the continuously-updated setpoints
        aimDynamicCommand(
            () -> targetShooterSpeed,
            () -> targetTurretAngle,
            () -> targetHoodAngle
        )
    ).withName("Superstructure.visionAimAtHub");
  }
  
  /**
   * Container for aiming calculation results
   */
  private static class AimingParameters {
    public final Angle turretAngle;
    public final Angle hoodAngle;
    public final AngularVelocity shooterSpeed;
    public final double distanceToHub;
    
    public AimingParameters(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterSpeed, double distanceToHub) {
      this.turretAngle = turretAngle;
      this.hoodAngle = hoodAngle;
      this.shooterSpeed = shooterSpeed;
      this.distanceToHub = distanceToHub;
    }
  }
  
  /**
   * Calculates the turret angle, hood angle, and shooter speed needed to hit the hub.
   * 
   * PHYSICS TIME!
   * This uses projectile motion equations to figure out the correct angles.
   * 
   * @param robotPos Our robot's position on the field
   * @param robotHeading Our robot's current rotation
   * @param hubPos The 3D position of the hub
   * @return Calculated aiming parameters
   */
  private AimingParameters calculateAimingParameters(
      Translation2d robotPos,
      Rotation2d robotHeading,
      Translation3d hubPos) {
    
    // Calculate vector from robot to hub
    Translation2d hubPos2d = hubPos.toTranslation2d();
    Translation2d robotToHub = hubPos2d.minus(robotPos);
    
    // Calculate horizontal distance to hub (for hood angle calculation)
    double horizontalDistance = robotToHub.getNorm();
    
    // Calculate vertical distance (height difference)
    // Assuming robot shooter is at some height above ground
    double shooterHeight = 0.6; // meters (adjust based on your robot)
    double verticalDistance = hubPos.getZ() - shooterHeight;
    
    // Calculate TURRET angle (relative to robot heading)
    // This is the angle the turret needs to rotate to point at the hub
    Rotation2d angleToHub = robotToHub.getAngle();
    Rotation2d turretAngle = angleToHub.minus(robotHeading);
    
    // Wrap angle to [-180, 180] degrees
    double turretDegrees = turretAngle.getDegrees();
    while (turretDegrees > 180) turretDegrees -= 360;
    while (turretDegrees < -180) turretDegrees += 360;
    
    // Calculate HOOD angle using projectile motion
    // This is more complex - we need to account for:
    // - Horizontal distance
    // - Vertical distance (height of hub)
    // - Initial velocity of the projectile
    
    // For now, use a simplified lookup table approach
    // You can replace this with actual projectile physics if needed
    Angle hoodAngle = calculateHoodAngleFromDistance(horizontalDistance, verticalDistance);
    
    // Calculate SHOOTER speed based on distance
    // Farther away = need more speed
    AngularVelocity shooterSpeed = calculateShooterSpeedFromDistance(horizontalDistance);
    
    return new AimingParameters(
        Degrees.of(turretDegrees),
        hoodAngle,
        shooterSpeed,
        horizontalDistance
    );
  }
  
  /**
   * Calculates the hood angle needed for a given distance.
   * 
   * LOOKUP TABLE APPROACH:
   * This uses a simple linear interpolation based on testing.
   * You should calibrate these values by:
   * 1. Shooting from different distances
   * 2. Recording what hood angle works best
   * 3. Updating these numbers
   * 
   * @param horizontalDist Distance to target in meters
   * @param verticalDist Height difference in meters
   * @return Hood angle in degrees
   */
  private Angle calculateHoodAngleFromDistance(double horizontalDist, double verticalDist) {
    // Simple lookup table (TUNE THESE VALUES!)
    // Distance (m) -> Hood Angle (degrees)
    // Close range: lower angle
    // Far range: higher angle
    
    if (horizontalDist < 2.0) {
      return Degrees.of(20);  // Close shots - flatter trajectory
    } else if (horizontalDist < 4.0) {
      return Degrees.of(35);  // Medium shots
    } else if (horizontalDist < 6.0) {
      return Degrees.of(45);  // Far shots
    } else {
      return Degrees.of(60);  // Very far - lob it!
    }
    
    // TODO: Replace with actual projectile physics equation:
    // θ = arctan((v² ± sqrt(v⁴ - g(gx² + 2yv²))) / (gx))
    // where v = launch velocity, g = gravity, x = horizontal dist, y = vertical dist
  }
  
  /**
   * Calculates shooter speed based on distance to target.
   * 
   * @param distance Distance to target in meters
   * @return Shooter speed in RPM
   */
  private AngularVelocity calculateShooterSpeedFromDistance(double distance) {
    // Base speed + extra for distance
    // TUNE THESE VALUES!
    double baseRPM = 3000;  // Minimum shooter speed
    double rpmPerMeter = 200; // Extra RPM per meter of distance
    
    double targetRPM = baseRPM + (distance * rpmPerMeter);
    
    // Clamp to reasonable limits
    targetRPM = Math.max(2500, Math.min(5000, targetRPM));
    
    return RPM.of(targetRPM);
  }

  /**
   * Stops all mechanisms - shooter stops spinning, turret and hood hold position.
   */
  public Command stopAllCommand() {
    return Commands.parallel(
        shooter.stop().asProxy(),
        turret.set(0).asProxy(),
        hood.set(0).asProxy()).withName("Superstructure.stopAll");
  }

  /**
   * Aims the superstructure to specific targets - used for auto-targeting.
   *
   * @param shooterSpeed Target shooter speed
   * @param turretAngle  Target turret angle
   * @param hoodAngle    Target hood angle
   */
  public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    return Commands.runOnce(() -> {
      targetShooterSpeed = shooterSpeed;
      targetTurretAngle = turretAngle;
      targetHoodAngle = hoodAngle;
    }).andThen(
        Commands.parallel(
            // shooter.setSpeed(shooterSpeed).asProxy(),
            turret.setAngle(turretAngle).asProxy(),
            hood.setAngle(hoodAngle).asProxy()))
        .withName("Superstructure.aim");
  }

  public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    targetShooterSpeed = shooterSpeed;
    targetTurretAngle = turretAngle;
    targetHoodAngle = hoodAngle;
  }

  /**
   * Aims the superstructure using suppliers - useful for dynamic targeting.
   *
   * @param shooterSpeedSupplier Supplier for target shooter speed
   * @param turretAngleSupplier  Supplier for target turret angle
   * @param hoodAngleSupplier    Supplier for target hood angle
   */
  public Command aimDynamicCommand(
      Supplier<AngularVelocity> shooterSpeedSupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier) {
    return Commands.parallel(
        shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
        turret.setAngleDynamic(turretAngleSupplier).asProxy(),
        hood.setAngleDynamic(hoodAngleSupplier).asProxy())
        .withName("Superstructure.aimDynamic");
  }

  /**
   * Waits until the superstructure is ready to shoot.
   */
  public Command waitUntilReadyCommand() {
    return Commands.waitUntil(isReadyToShoot).withName("Superstructure.waitUntilReady");
  }

  /**
   * Aims and waits until ready - combines aim and wait.
   */
  public Command aimAndWaitCommand(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    return aimDynamicCommand(() -> shooterSpeed, () -> turretAngle, () -> hoodAngle)
        .andThen(waitUntilReadyCommand())
        .withName("Superstructure.aimAndWait");
  }

  /**
   * Auto aim command for driver controls
   */
  public Command autoAimCommand() {
    return aimCommand(targetShooterSpeed, targetTurretAngle, targetHoodAngle);
  }

  /**
   * Manual turret control for driver controls
   */
  public Command turretManualCommand(double speed) {
    return turret.set(speed).withName("Superstructure.turretManual");
  }

  public Command setTurretForward() {
    return turret.setAngle(Degrees.of(0)).withName("Superstructure.setTurretForward");
  }

  public Command setTurretLeft() {
    return turret.setAngle(Degrees.of(45)).withName("Superstructure.setTurretLeft");
  }

  public Command setTurretRight() {
    return turret.setAngle(Degrees.of(-45)).withName("Superstructure.setTurretRight");
  }

  // Getters for current state
  public AngularVelocity getShooterSpeed() {
    return shooter.getSpeed();
  }

  public Angle getTurretAngle() {
    return turret.getRawAngle();
  }

  public Angle getHoodAngle() {
    return hood.getAngle();
  }

  public AngularVelocity getTargetShooterSpeed() {
    return targetShooterSpeed;
  }

  public Angle getTargetTurretAngle() {
    return targetTurretAngle;
  }

  public Angle getTargetHoodAngle() {
    return targetHoodAngle;
  }

  public Translation3d getAimPoint() {
    return aimPoint;
  }

  public void setAimPoint(Translation3d newAimPoint) {
    this.aimPoint = newAimPoint;
  }

  public Rotation3d getAimRotation3d() {
    // See
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    return new Rotation3d(
        Degrees.of(0), // no roll 🤞
        hood.getAngle().unaryMinus(), // pitch is negative hood angle
        turret.getRobotAdjustedAngle());
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return intake.intakeCommand().withName("Superstructure.intake");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return intake.ejectCommand().withName("Superstructure.eject");
  }

  /**
   * Command to run the hopper forward while held.
   */
  public Command hopperFeedCommand() {
    return hopper.feedCommand().withName("Superstructure.feed");
  }

  /**
   * Command to run the hopper in reverse while held.
   */
  public Command hopperReverseCommand() {
    return hopper.reverseCommand().withName("Superstructure.hopperReverse");
  }

  /**
   * Command to run the kicker forward while held, stops when released.
   */
  public Command kickerFeedCommand() {
    return kicker.feedCommand().withName("Superstructure.kickerFeed");
  }

  /**
   * Command to run the kicker stop while held, stops when released.
   */
  public Command kickerStopCommand() {
    return kicker.stopCommand().withName("Superstructure.kickerStop");
  }

  public Command feedAllCommand() {
    return Commands.parallel(
        hopper.feedCommand().asProxy(),
        kicker.feedCommand().asProxy()).withName("Superstructure.feedAll");
    // intake.setPivotAngle(Degrees.of(46)).asProxy()).withName("Superstructure.feedAll");
  }

  public Command backFeedAllCommand() {
    return Commands.parallel(
        hopper.backFeedCommand().asProxy(),
        intake.backFeedAndRollCommand().asProxy()).withName("Superstructure.backFeedAll");
  }

  public Command intakeBounceCommand() {
  return Commands.sequence(
  Commands.runOnce(() -> intake.setPivotAngle(Degrees.of(115))).asProxy()
  .withName("Superstructure.intakeBounce.deploy"),
  Commands.waitSeconds(0.5),
  Commands.runOnce(() -> intake.setPivotAngle(Degrees.of(59))).asProxy()
  .withName("Superstructure.intakeBounce.feed"),
  Commands.waitSeconds(0.5))
  .withName("Superstructure.intakeBounce");
  }

  public Command stopFeedingAllCommand() {
    return Commands.parallel(
        hopper.stopCommand().asProxy(),
        kicker.stopCommand().asProxy());
        //intake.deployAndRollCommand().asProxy()).withName("Superstructure.stopFeedingAll");
  }

  /**
   * Command to set the intake pivot angle.
   */
  public Command setIntakePivotAngle(Angle angle) {
    return intake.setPivotAngle(angle).withName("Superstructure.setIntakePivotAngle");
  }

  public Command setIntakeDeployAndRoll() {
    return intake.deployAndRollCommand().withName("Superstructure.setIntakeDeployAndRoll");
  }

  /**
   * Command to shoot - spins up shooter.
   */
  public Command shootCommand() {
    // return shooter.sysId().withName("Superstructure.shoot");
    return shooter.spinUp().withName("Superstructure.shoot");
  }

  /**
   * Command to stop shooting - stops shooter.
   */
  public Command stopShootingCommand() {
    return shooter.stop().withName("Superstructure.stopShooting");
  }

  // Re-zero intake pivot if needed
  public Command rezeroIntakePivotAndTurretCommand() {
    return Commands.parallel(
        turret.rezero().withName("Superstructure.rezeroTurret"),
        intake.rezero().withName("Superstructure.rezeroIntakePivot"))
        .withName("Superstructure.rezeroIntakePivotAndTurret");
  }

  @Override
  public void periodic() {
    // Superstructure doesn't need periodic updates - subsystems handle their own

    String shooterOut = "S:" + isShooterAtSpeed.getAsBoolean() + "(" + Math.round(shooter.getSpeed().in(RPM)) + "/"
        + Math.round(targetShooterSpeed.in(RPM)) + ")";

    String turretOut = "T:" + isTurretOnTarget.getAsBoolean() + "(" + Math.round(turret.getRawAngle().in(Degrees)) + "/"
        + Math.round(targetTurretAngle.in(Degrees)) + ")";

    String hoodOut = "H:" + isHoodOnTarget.getAsBoolean() + "(" + Math.round(hood.getAngle().in(Degrees)) + "/"
        + Math.round(targetHoodAngle.in(Degrees)) + ")";

    String readyOut = "R:" + isReadyToShoot.getAsBoolean();

    // System.out.println(shooterOut + " " + turretOut + " " + hoodOut + " " +
    // readyOut);
  }

  public Command useRequirement() {
    return runOnce(() -> {
    });
  }

  public Pose3d getShooterPose() {
    // Position of the shooter relative to the "front" of the robot. Rotation
    // element is based on hood and turret angles
    return new Pose3d(new Translation3d(
        Meter.of(-0.3),
        Meter.of(0),
        Meter.of(0.6)),
        getAimRotation3d());
  }

  public LinearVelocity getTangentialVelocity() {
    return shooter.getTangentialVelocity();
  }
}
