package frc.robot.controls;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.maplesim.RebuiltFuelOnFly;

public class DriverControls {

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
    
    // Create swerve request for field-centric drive
    // Configure field-centric request (avoid DriveRequestType API mismatch)
    SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDeadband(maxSpeed * ControllerConstants.DEADBAND)
        .withRotationalDeadband(maxAngularRate * ControllerConstants.DEADBAND);
    // Set default drive command with speed scaling (0.25 = 25% speed)
    double speedScale = 0.25; // TODO: Tune speed scaling
    
    drivetrain.getDrivetrain().setDefaultCommand(
        drivetrain.getDrivetrain().applyRequest(() -> 
            fieldCentric
                .withVelocityX(-controller.getLeftY() * maxSpeed * speedScale)
                .withVelocityY(-controller.getLeftX() * maxSpeed * speedScale)
                .withRotationalRate(-controller.getRightX() * maxAngularRate * speedScale)
        ).withName("Drive.FieldCentric")
    );

    if (DriverStation.isTest()) {
      // Test mode controls
      controller.b().whileTrue(Commands.runOnce(() -> {
        // Center modules command equivalent
        drivetrain.getDrivetrain().setControl(new SwerveRequest.Idle());
      }, drivetrain.getDrivetrain()));
      
      controller.x().whileTrue(Commands.runOnce(() -> {
        drivetrain.getDrivetrain().setControl(new SwerveRequest.SwerveDriveBrake());
      }, drivetrain.getDrivetrain()).repeatedly());
      
      controller.y().onTrue(drivetrain.zeroGyro());

      // SysId commands
      controller.start().whileTrue(drivetrain.getDrivetrain().sysIdDynamic(
          edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
      controller.back().whileTrue(drivetrain.getDrivetrain().sysIdQuasistatic(
          edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
          
    // } else if (Robot.isSimulation()) {
    //   // Fire fuel 10 times per second while button is held
    //   controller.back().whileTrue(
    //       Commands.run(() -> {
    //         Pose3d shooterPosition = new Pose3d();
    //         try {
    //           shooterPosition = superstructure.getShooterPose();
    //         } catch (Exception e) {
    //           // If shooter pose not available, use robot pose
    //           shooterPosition = drivetrain.getPose3d();
    //         }

    //         GamePieceProjectile projectile = new RebuiltFuelOnFly(
    //             "Fuel-Test",
    //             shooterPosition,
    //             superstructure.getShooterLinearVelocity());

    //         SimulatedArena.getInstance().addGamePiece(projectile);
    //       })
    //           .withName("Test.ShootFuel"));
    }

    // Intake controls
    controller.rightTrigger().whileTrue(superstructure.intakeCommand());
    controller.leftTrigger().whileTrue(superstructure.ejectCommand());

    // Shooter controls  
    controller.rightBumper().whileTrue(superstructure.shootCommand());
    
    // VISION AIMING - This is the new feature!
    // Hold left bumper to automatically aim at the hub using vision
    // The robot will:
    // 1. Use AprilTags to locate the hub
    // 2. Calculate the correct turret and hood angles
    // 3. Adjust shooter speed based on distance
    // 4. Continuously track the target as you drive
    controller.leftBumper().whileTrue(
        superstructure.visionAimAtHubCommand()
            .withName("Driver.VisionAimAtHub")
    );

    // Position controls
    // controller.a().whileTrue(superstructure.stowCommand());
    // controller.b().whileTrue(superstructure.scoreLowCommand());
    // controller.y().whileTrue(superstructure.scoreHighCommand());

    // Manual turret control
    controller.povUp().whileTrue(superstructure.turretManualCommand(0.2));
    controller.povDown().whileTrue(superstructure.turretManualCommand(-0.2));
  }
}
