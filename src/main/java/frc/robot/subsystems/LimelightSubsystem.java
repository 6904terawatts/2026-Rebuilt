package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * Simple Limelight AprilTag Lock System
 * 
 * Locks to specific AprilTags with offset positioning (~4 feet from hub).
 * Uses POV buttons for quick tag selection.
 */
public class LimelightSubsystem extends SubsystemBase {
  
    
  private final SwerveSubsystem swerveSubsystem;
  
  // ============================================================================
  // HUB APRILTAG IDS
  // ============================================================================
  
  // RED ALLIANCE HUB TAGS
  private static final int[] RED_HUB_TAGS = {8, 9, 10, 11};
  
  // BLUE ALLIANCE HUB TAGS
  private static final int[] BLUE_HUB_TAGS = {24, 25, 26, 27};
  
  // Offset distance from hub (meters) - ~4 feet
  private static final double OFFSET_DISTANCE_METERS = 1.22;  // 4 feet = 1.22 meters
  
  // PID constants for alignment
  private static final double ROTATION_KP = 0.05;  // Rotation speed multiplier
  private static final double FORWARD_SPEED = 0.3;  // Forward speed when approaching
  private static final double BACKUP_SPEED = -0.2;  // Backup speed when too close
  
  // Distance thresholds (target area %)
  private static final double TOO_FAR_THRESHOLD = 2.0;   // < 2% area = too far
  private static final double TOO_CLOSE_THRESHOLD = 3.5; // > 3.5% area = too close
  
  private boolean isAligning = false;
  
  public LimelightSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }
 
  /**
   * Get current alliance
   */
  private Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }
 
  /**
   * Get the AprilTag IDs for current alliance's hub
   */
  private int[] getHubTags() {
    return (getAlliance() == Alliance.Blue) ? BLUE_HUB_TAGS : RED_HUB_TAGS;
  }
 
  /**
   * Check if Limelight is seeing a hub AprilTag
   */
  public boolean hasHubTarget() {
    if (!LimelightHelpers.getTV("limelight")) {
      return false;
    }
    
    // Get the tag ID that Limelight sees
    int tagId = (int) LimelightHelpers.getFiducialID("limelight");
    
    // Check if it's one of our hub tags
    int[] hubTags = getHubTags();
    for (int hubTag : hubTags) {
      if (tagId == hubTag) {
        return true;
      }
    }
    
    return false;
  }
 
  /**
   * Start auto-alignment to hub
   * 
   * Just hold the button - robot will:
   * 1. Find any hub AprilTag
   * 2. Rotate to center on it
   * 3. Drive to ~4 feet away
   * 4. Hold position
   */
  public Command autoAlign() {
    return Commands.run(() -> {
      isAligning = true;
      
      // Check if we can see a hub tag
      if (!hasHubTarget()) {
        System.out.println("⚠️ No hub AprilTag visible!");
        // Stop moving if no target
        swerveSubsystem.getDrivetrain().setControl(
            new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        return;
      }
      
      // Get Limelight data
      double tx = LimelightHelpers.getTX("limelight");  // Horizontal offset (degrees)
      double ty = LimelightHelpers.getTY("limelight");  // Vertical offset (degrees)
      double ta = LimelightHelpers.getTA("limelight");  // Target area (0-100%)
      
      // Calculate rotation to center on tag
      double rotationSpeed = -tx * ROTATION_KP;
      
      // Calculate forward/backward speed based on distance
      double forwardSpeed = 0;
      
      if (ta < TOO_FAR_THRESHOLD) {
        // Too far - move forward
        forwardSpeed = FORWARD_SPEED;
      } else if (ta > TOO_CLOSE_THRESHOLD) {
        // Too close - back up
        forwardSpeed = BACKUP_SPEED;
      } else {
        // Just right - hold position
        forwardSpeed = 0;
      }
      
      // Apply swerve drive control
      swerveSubsystem.getDrivetrain().setControl(
          new SwerveRequest.RobotCentric()
              .withVelocityX(forwardSpeed)   // Forward/backward
              .withVelocityY(0)              // No strafe
              .withRotationalRate(rotationSpeed)  // Rotate to center
      );
      
      // Log current tag being tracked
      int currentTag = (int) LimelightHelpers.getFiducialID("limelight");
      SmartDashboard.putNumber("AutoAlign/Current Tag", currentTag);
      SmartDashboard.putBoolean("AutoAlign/At Distance", 
          ta >= TOO_FAR_THRESHOLD && ta <= TOO_CLOSE_THRESHOLD);
      SmartDashboard.putBoolean("AutoAlign/Centered", Math.abs(tx) < 2.0);
      
    }, swerveSubsystem.getDrivetrain()).finallyDo(() -> {
      isAligning = false;
      System.out.println("🛑 Auto-align stopped");
    }).withName("AutoAlign.ToHub");
  }
 
  /**
   * Stop alignment
   */
  public Command stopAlign() {
    return Commands.runOnce(() -> {
      isAligning = false;
      System.out.println("🛑 Stopped auto-align");
    }).withName("AutoAlign.Stop");
  }
 
  /**
   * Check if currently aligning
   */
  public boolean isAligning() {
    return isAligning;
  }
 
  @Override
  public void periodic() {
    // Update telemetry
    SmartDashboard.putString("AutoAlign/Alliance", getAlliance().toString());
    SmartDashboard.putBoolean("AutoAlign/Has Hub Target", hasHubTarget());
    SmartDashboard.putBoolean("AutoAlign/Is Aligning", isAligning);
    SmartDashboard.putBoolean("AutoAlign/Has Target", LimelightHelpers.getTV("limelight"));
    SmartDashboard.putNumber("AutoAlign/TX", LimelightHelpers.getTX("limelight"));
    SmartDashboard.putNumber("AutoAlign/TY", LimelightHelpers.getTY("limelight"));
    SmartDashboard.putNumber("AutoAlign/TA", LimelightHelpers.getTA("limelight"));
    
    if (LimelightHelpers.getTV("limelight")) {
      SmartDashboard.putNumber("AutoAlign/Tag ID", LimelightHelpers.getFiducialID("limelight"));
    }
  }
}