package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.VisionSubsystem.VisionMeasurement;

/**
 * SwerveSubsystem - CTRE Phoenix 6 Swerve Drive with Vision Integration
 * 
 * VISION FUSION:
 * This subsystem combines TWO sources of position data:
 * 1. Wheel Odometry - measures wheel rotations to track movement
 * 2. Vision - uses AprilTags to determine absolute position on field
 * 
 * By combining both, we get:
 * - Smooth, fast updates from wheels (odometry)
 * - Absolute accuracy from vision (corrects drift)
 * - Best of both worlds!
 * 
 * IMPORTANT: This extends SubsystemBase so periodic() is called automatically!
 */
public class SwerveSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    public final Field2d field = new Field2d(); // For field visualization
    
    // Vision subsystem for AprilTag tracking
    // This is Optional because vision might not always be available
    private Optional<VisionSubsystem> visionSubsystem = Optional.empty();
    
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
        
        // Configure PathPlanner for autonomous
        configurePathPlanner();
    }
    
    /**
     * Configure PathPlanner for autonomous path following.
     * 
     * This sets up AutoBuilder so PathPlanner can create autonomous routines.
     * Must be called before using AutoBuilder.buildAutoChooser()
     */
    private void configurePathPlanner() {
        try {
            // Load the robot configuration from the PathPlanner GUI settings
            RobotConfig config = RobotConfig.fromGUISettings();
            
            // Configure AutoBuilder with all the necessary callbacks
            AutoBuilder.configure(
                this::getPose,              // Function to get current robot pose
                this::resetPose,            // Function to reset odometry
                this::getChassisSpeeds,     // Function to get current chassis speeds
                (speeds, feedforwards) -> {
                    // Use FieldCentricFacingAngle to apply the speeds
                    // CTRE doesn't have a direct ApplyChassisSpeeds, so we convert
                    drivetrain.setControl(
                        new SwerveRequest.FieldCentric()
                            .withVelocityX(speeds.vxMetersPerSecond)
                            .withVelocityY(speeds.vyMetersPerSecond)
                            .withRotationalRate(speeds.omegaRadiansPerSecond)
                    );
                },                          // Function to drive the robot
                new PPHolonomicDriveController(
                    // PID constants for translation (X and Y)
                    new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0),
                    // PID constants for rotation
                    new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0)
                ),
                config,                     // Robot configuration
                () -> {
                    // Should flip path based on alliance color
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this                        // This subsystem as a requirement
            );
        } catch (Exception e) {
            // If PathPlanner config fails, log the error but don't crash
            DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(), e.getStackTrace());
        }
    }
    
    /**
     * Sets the vision subsystem for pose fusion.
     * 
     * WHY THIS IS SEPARATE:
     * The VisionSubsystem is created in RobotContainer AFTER this SwerveSubsystem,
     * so we can't pass it in the constructor. Instead, we set it later.
     * 
     * @param vision The VisionSubsystem to use for pose estimation
     */
    public void setVisionSubsystem(VisionSubsystem vision) {
        this.visionSubsystem = Optional.of(vision);
        Logger.recordOutput("Swerve/VisionEnabled", true);
    }
    
    /**
     * Called every 20ms to update robot state.
     * 
     * VISION FUSION HAPPENS HERE:
     * 1. Check if we have a vision measurement
     * 2. If yes, add it to the pose estimator
     * 3. The pose estimator fuses it with wheel odometry
     * 4. Result: More accurate robot position!
     * 
     * ALSO LOGS: All swerve data for AdvantageScope visualization
     */
    @Override
    public void periodic() {
        // ====================================================================
        // LOG BASIC SWERVE DATA
        // ====================================================================
    
        // Current robot pose (position + rotation)
        Pose2d currentPose = getPose();
        Logger.recordOutput("Swerve/Pose", currentPose);
        Logger.recordOutput("Swerve/Pose3d", getPose3d());
        
        // Robot velocities
        ChassisSpeeds speeds = getChassisSpeeds();
        Logger.recordOutput("Swerve/VelocityX", speeds.vxMetersPerSecond);
        Logger.recordOutput("Swerve/VelocityY", speeds.vyMetersPerSecond);
        Logger.recordOutput("Swerve/VelocityOmega", speeds.omegaRadiansPerSecond);
        
        // Robot heading (rotation)
        Logger.recordOutput("Swerve/Heading", currentPose.getRotation().getDegrees());
        
// Module states (for detailed debugging)
var moduleStates = drivetrain.getState().ModuleStates;
Logger.recordOutput("Swerve/ModuleStates", moduleStates);

field.setRobotPose(currentPose);
SmartDashboard.putData(field);


        // ====================================================================
        // VISION FUSION
        // ====================================================================
        
        // Only fuse vision data if we have a vision subsystem
        visionSubsystem.ifPresent(vision -> {
            // Get the latest vision measurement
            Optional<VisionMeasurement> measurement = vision.getLatestVisionMeasurement();
            
            // If we have a valid vision measurement, add it to pose estimation
            measurement.ifPresent(visionData -> {
                
                // Calculate standard deviations (trust level) for the vision measurement
                // More tags = more trust (lower standard deviation)
                // Further from tags = less trust (higher standard deviation)
                Matrix<N3, N1> visionStdDevs = calculateVisionStdDevs(visionData.tagCount);
                
                // Add the vision measurement to the drivetrain's pose estimator
                // This fuses it with wheel odometry for a better position estimate
                drivetrain.addVisionMeasurement(
                    visionData.pose,           // Where vision thinks we are
                    visionData.timestamp,      // When the measurement was taken
                    visionStdDevs             // How much to trust this measurement
                );
                
                // Log for debugging in AdvantageScope
                Logger.recordOutput("Swerve/VisionPoseUsed", visionData.pose);
                Logger.recordOutput("Swerve/VisionTagCount", visionData.tagCount);
                Logger.recordOutput("Swerve/VisionStdDevXY", visionStdDevs.get(0, 0));
                Logger.recordOutput("Swerve/VisionStdDevTheta", visionStdDevs.get(2, 0));
            });
        });
    }
    
    /**
     * Calculates how much to trust a vision measurement.
     * 
     * STANDARD DEVIATIONS EXPLAINED:
     * Think of this as "error bars" on a measurement.
     * - Small std dev = high trust (measurement is probably accurate)
     * - Large std dev = low trust (measurement might be off)
     * 
     * FACTORS THAT AFFECT TRUST:
     * - Number of tags: More tags = more trust
     * - Distance to tags: Closer = more trust
     * - Quality of detection: Clear view = more trust
     * 
     * RETURNED VALUES:
     * [X std dev, Y std dev, Rotation std dev]
     * Units are in meters and radians
     * 
     * @param numTags Number of AprilTags used in the measurement
     * @return 3x1 matrix of standard deviations
     */
    private Matrix<N3, N1> calculateVisionStdDevs(int numTags) {
        // Base standard deviation values (in meters and radians)
        // These were tuned experimentally for good fusion
        double xyStdDev;
        double thetaStdDev;
        
        if (numTags >= 2) {
            // Multiple tags: high trust
            // Position error: ~3cm, Rotation error: ~5 degrees
            xyStdDev = 0.03;    // 3 centimeters
            thetaStdDev = 0.087; // 5 degrees in radians
        } else {
            // Single tag: medium trust
            // Position error: ~10cm, Rotation error: ~10 degrees
            xyStdDev = 0.1;     // 10 centimeters  
            thetaStdDev = 0.174; // 10 degrees in radians
        }
        
        // Return as a 3x1 matrix [X, Y, Theta]
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
    
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

     public double getTurnRate() {
    return Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond);
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
}