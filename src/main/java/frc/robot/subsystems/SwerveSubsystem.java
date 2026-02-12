package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;

/**
 * SwerveSubsystem - CTRE Phoenix 6 Swerve Drive
 * Completely removed YAGSL - using only CTRE CommandSwerveDrivetrain
 */
public class SwerveSubsystem {
    private final CommandSwerveDrivetrain drivetrain;
    public final Field2d field = new Field2d(); // For field visualization
    
    // Constants from your TunerConstants
    private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond);
    private static final double MAX_ANGULAR_RATE = Math.PI * 2; // radians per second
    
    // Swerve requests for teleop control
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDeadband(MAX_SPEED * 0.1)
        .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MAX_SPEED * 0.1)
        .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    
    public SwerveSubsystem() {
        // Create the drivetrain using your TunerConstants
        drivetrain = TunerConstants.createDrivetrain();
        
        //configurePathPlanner();
    }
    
    /**
     * Configure PathPlanner for auto
     */
    // private void configurePathPlanner() {
    //     try {
    //         RobotConfig config = RobotConfig.fromGUISettings();
            
    //         AutoBuilder.configure(
    //             this::getPose,
    //             this::resetPose,
    //             this::getChassisSpeeds,
    //             this::setChassisSpeeds,
    //             new PPHolonomicDriveController(
    //                 config.robotConfig.translationConstants,
    //                 config.robotConfig.rotationConstants
    //             ),
    //             config,
    //             () -> {
    //                 var alliance = DriverStation.getAlliance();
    //                 return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    //             },
    //             drivetrain
    //         );
    //     } catch (Exception e) {
    //         DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(), e.getStackTrace());
    //     }
    // }
    
    /**
     * Get current robot pose
     */
    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }
    
    /**
     * Get current robot pose in 3D
     */
    public Pose3d getPose3d() {
        Pose2d pose2d = getPose();
        return new Pose3d(
            pose2d.getX(),
            pose2d.getY(),
            0.0,
            new Rotation3d(0, 0, pose2d.getRotation().getRadians())
        );
    }
    
    /**
     * Reset robot pose
     */
    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }
    
    /**
     * Get current chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return drivetrain.getState().Speeds;
    }
    
    /**
     * Set chassis speeds (for PathPlanner)
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        drivetrain.setControl(robotCentric
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
    }
    
    /**
     * Command to drive with joystick inputs (field-centric)
     */
    public Command driveCommand(
        java.util.function.DoubleSupplier translationX,
        java.util.function.DoubleSupplier translationY,
        java.util.function.DoubleSupplier rotation) {
        
        return Commands.run(() -> {
            double xSpeed = translationX.getAsDouble() * MAX_SPEED;
            double ySpeed = translationY.getAsDouble() * MAX_SPEED;
            double rotSpeed = rotation.getAsDouble() * MAX_ANGULAR_RATE;
            
            drivetrain.setControl(fieldCentric
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotSpeed));
        }, drivetrain);
    }
    
    /**
     * Command to drive forward for testing
     */
    public Command driveForward() {
        return Commands.run(() -> {
            drivetrain.setControl(robotCentric
                .withVelocityX(1.0) // 1 m/s forward
                .withVelocityY(0)
                .withRotationalRate(0));
        }, drivetrain);
    }
    
    /**
     * Command to drive backward for testing
     */
    public Command driveBackwards() {
        return Commands.run(() -> {
            drivetrain.setControl(robotCentric
                .withVelocityX(-1.0) // 1 m/s backward
                .withVelocityY(0)
                .withRotationalRate(0));
        }, drivetrain);
    }
    
    /**
     * Lock wheels in X pattern
     */
    public Command lock() {
        return Commands.runOnce(() -> {
            drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        }, drivetrain);
    }
    
    /**
     * Zero gyro heading
     */
    public Command zeroGyro() {
        return Commands.runOnce(() -> {
            drivetrain.seedFieldCentric();
        }, drivetrain);
    }
    
    /**
     * Get the actual CTRE drivetrain
     */
    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }
    
    /**
     * Compatibility class for constants
     */
    private static class Constants {
        public static final double MAX_SPEED = SwerveSubsystem.MAX_SPEED;
        public static final double MAX_ANGULAR_RATE = SwerveSubsystem.MAX_ANGULAR_RATE;
    }
}
