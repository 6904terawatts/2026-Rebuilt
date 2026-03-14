


package frc.robot.controls;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.maplesim.RebuiltFuelOnFly;

public class DriverControls {

  // Speed mode multipliers
  private static final double SLOW_MODE_SCALE = 0.3;    // 30% - precision
  private static final double NORMAL_MODE_SCALE = 0.6;  // 60% - default
  private static final double TURBO_MODE_SCALE = 1.0;   // 100% - max speed
  
  // Exponential curve for smoother control
  private static final double STICK_EXPONENT = 1.6;

  private static Pose2d getTargetPose() {
    Pose2d hubPose = new Pose2d(
        Meter.of(11.902),
        Meter.of(4.031),
        Rotation2d.kZero);

    Logger.recordOutput("DriverControls/TargetHubPose", hubPose);
    return hubPose;
  }

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // Get max speeds from TunerConstants
    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double maxAngularRate = Math.PI * 2; // 2 rotations per second
    
    // Speed mode tracker (starts in normal mode)
    final double[] currentSpeedScale = {NORMAL_MODE_SCALE};
    
    // Create swerve request for field-centric drive
    SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * ControllerConstants.DEADBAND)
        .withRotationalDeadband(maxAngularRate * ControllerConstants.DEADBAND);
    
    // ============================================================================
    // DEFAULT DRIVE COMMAND - Smooth with exponential curve
    // ============================================================================
    
    drivetrain.getDrivetrain().setDefaultCommand(
        drivetrain.getDrivetrain().applyRequest(() -> {
            // Get raw stick inputs
            double rawLeftY = controller.getLeftY();
            double rawLeftX = controller.getLeftX();
            double rawRightX = -controller.getRightX();
            
            // Apply deadband
            rawLeftY = MathUtil.applyDeadband(rawLeftY, ControllerConstants.DEADBAND);
            rawLeftX = MathUtil.applyDeadband(rawLeftX, ControllerConstants.DEADBAND);
            rawRightX = MathUtil.applyDeadband(rawRightX, ControllerConstants.DEADBAND);
            
            // Apply exponential curve for smoother control
            double smoothLeftY = applyExponentialCurve(rawLeftY);
            double smoothLeftX = applyExponentialCurve(rawLeftX);
            double smoothRightX = applyExponentialCurve(rawRightX);
            
            // Apply speed scaling
            double vx = smoothLeftY * maxSpeed * currentSpeedScale[0];
            double vy = smoothLeftX * maxSpeed * currentSpeedScale[0];
            double omega = smoothRightX * maxAngularRate * currentSpeedScale[0];
            
            // Log current speed mode
            Logger.recordOutput("Drive/SpeedMode", 
                currentSpeedScale[0] == SLOW_MODE_SCALE ? "SLOW" :
                currentSpeedScale[0] == TURBO_MODE_SCALE ? "TURBO" : "NORMAL");
            
            return fieldCentric
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega);
        }).withName("Drive.FieldCentric")
    );

    // ============================================================================
    // TEST MODE CONTROLS
    // ============================================================================
    
    if (DriverStation.isTest()) {
      // Test mode controls
      controller.b().whileTrue(Commands.runOnce(() -> {
        drivetrain.getDrivetrain().setControl(new SwerveRequest.Idle());
      }, drivetrain.getDrivetrain()));
      
      controller.x().whileTrue(Commands.runOnce(() -> {
        drivetrain.getDrivetrain().setControl(new SwerveRequest.SwerveDriveBrake());
      }, drivetrain.getDrivetrain()).repeatedly());
      
      controller.y().onTrue(drivetrain.zeroGyro());

      controller.start().whileTrue(drivetrain.getDrivetrain().sysIdDynamic(
          edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
      controller.back().whileTrue(drivetrain.getDrivetrain().sysIdQuasistatic(
          edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
    }

    // ============================================================================
    // SPEED MODES (Left/Right Bumper)
    // ============================================================================
    
    // Left Bumper = SLOW MODE (hold for precision)
    controller.leftBumper()
        .whileTrue(Commands.runOnce(() -> currentSpeedScale[0] = SLOW_MODE_SCALE))
        .onFalse(Commands.runOnce(() -> currentSpeedScale[0] = NORMAL_MODE_SCALE));
    
    // Right Bumper = TURBO MODE (hold for max speed)
    controller.rightBumper()
        .whileTrue(Commands.runOnce(() -> currentSpeedScale[0] = TURBO_MODE_SCALE))
        .onFalse(Commands.runOnce(() -> currentSpeedScale[0] = NORMAL_MODE_SCALE));
    
    // ============================================================================
    // INTAKE CONTROLS (A/B Buttons)
    // ============================================================================
    
    // A Button - Intake UP (stow)
    controller.a().onTrue(
        superstructure.setIntakePivotAngle(Degrees.of(-144))
            .withName("Driver.IntakeUp")
    );
    
    // B Button - Intake DOWN (deploy)
    controller.b().onTrue(
        superstructure.setIntakePivotAngle(Degrees.of(0))
            .withName("Driver.IntakeDown")
    );
   
    
    // ============================================================================
    // VISION & LIMELIGHT (Back/X/Y)
    // ============================================================================
    
    // Back Button - Vision Aim at Hub
    controller.rightTrigger().whileTrue(
        superstructure.visionAimAtHubCommand()
            .withName("Driver.VisionAimAtHub")
    );
    
    // X Button - Limelight Tracking Drive (drive toward AprilTag)
    SwerveRequest.RobotCentric limelightDrive = new SwerveRequest.RobotCentric();
    controller.x().whileTrue(
        Commands.run(() ->
            drivetrain.getDrivetrain().setControl(
                limelightDrive
                    .withVelocityX(LimelightHelpers.getTY("limelight") * 0.1)
                    .withVelocityY(-LimelightHelpers.getTX("limelight") * 0.05)
                    .withRotationalRate(0)
            ),
            drivetrain.getDrivetrain()
        ).withName("Drive.LimelightTrack")
    );
    
    // Y Button - Reset Gyro
    controller.y().onTrue(drivetrain.zeroGyro());
    
    // ============================================================================
    // TRIGGERS (Intake/Eject)
    // ============================================================================
    
    // controller.rightTrigger().whileTrue(superstructure.intakeCommand());
    // controller.leftTrigger().whileTrue(superstructure.ejectCommand());
    
    // ============================================================================
    // D-PAD (Manual Turret Control)
    // ============================================================================
    
    controller.povUp().whileTrue(superstructure.turretManualCommand(0.2));
    controller.povDown().whileTrue(superstructure.turretManualCommand(-0.2));
    
    // ============================================================================
    // START BUTTON (Lock Wheels)
    // ============================================================================
    
    controller.start().whileTrue(
        Commands.run(() -> 
            drivetrain.getDrivetrain().setControl(new SwerveRequest.SwerveDriveBrake()),
            drivetrain.getDrivetrain()
        ).withName("Drive.LockWheels")
    );
  }
  
  // ============================================================================
  // HELPER METHODS
  // ============================================================================
  
  /**
   * Apply exponential curve to joystick input for smoother control.
   * Small movements = very small output (precise)
   * Large movements = proportional output (responsive)
   */
  private static double applyExponentialCurve(double input) {
      return Math.copySign(Math.pow(Math.abs(input), STICK_EXPONENT), input);
  }
}