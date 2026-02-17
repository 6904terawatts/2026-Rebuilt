package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * VisionSubsystem handles all vision tracking using the Limelight camera.
 * 
 * KEY CONCEPTS:
 * - The Limelight can track AprilTags (fiducial markers) on the field
 * - AprilTags are used to determine the robot's position on the field
 * - The hub has AprilTags that we can track to aim at it
 * - MegaTag2 uses multiple AprilTags simultaneously for better accuracy
 * 
 * HOW IT WORKS:
 * 1. The Limelight continuously scans for AprilTags
 * 2. It calculates the robot's position based on the tags it sees
 * 3. This "vision estimate" is fused with wheel odometry for accurate positioning
 * 4. We can also get the position of specific targets (like the hub) for aiming
 */
public class VisionSubsystem extends SubsystemBase {
  
  // ============================================================================
  // CONFIGURATION CONSTANTS
  // ============================================================================
  
  // Name of the Limelight (must match what's configured in the Limelight web interface)
  private static final String LIMELIGHT_NAME = "limelight";
  
  // Physical offset of the camera from the robot's center
  // These values define WHERE the camera is mounted on the robot
  // Format: (X forward/back, Y left/right, Z up/down) in meters
  // Rotation: (roll, pitch, yaw) in radians
  private static final Pose3d CAMERA_TO_ROBOT_OFFSET = new Pose3d(
      Inches.of(-11.00).in(Meters),   // 11 inches BACK from robot center
      Inches.of(-9.0).in(Meters),      // 9 inches to the LEFT
      Inches.of(12.75).in(Meters),     // 12.75 inches UP from ground
      new Rotation3d(
          0,                            // No roll (camera not tilted sideways)
          Degrees.of(21).in(Radians),  // Pitched up 21 degrees
          Degrees.of(180).in(Radians)  // Facing backwards (180 degrees)
      )
  );
  
  // AprilTag IDs for the hubs (these should match the field setup)
  // Different tags identify different targets on the field
  private static final int RED_HUB_TAG_ID = 4;    // Example ID - adjust based on field
  private static final int BLUE_HUB_TAG_ID = 7;   // Example ID - adjust based on field
  
  // Known positions of the hubs on the field (in meters)
  // These are fallback positions if we can't see the AprilTags
  private static final Pose3d RED_HUB_POSITION = new Pose3d(
      Meter.of(11.938),  // X position
      Meter.of(4.034536), // Y position
      Meter.of(1.5748),   // Z position (height)
      new Rotation3d()    // No rotation
  );
  
  private static final Pose3d BLUE_HUB_POSITION = new Pose3d(
      Meter.of(4.5974),
      Meter.of(4.034536),
      Meter.of(1.5748),
      new Rotation3d()
  );
  
  // Minimum number of AprilTags we need to see for a "good" pose estimate
  // More tags = more accurate position estimate
  private static final int MIN_TAGS_FOR_POSE = 1;
  
  // ============================================================================
  // STATE VARIABLES
  // ============================================================================
  
  // Whether vision is enabled (can be toggled for testing)
  private boolean visionEnabled = true;
  
  // The latest vision estimate of the robot's position
  private Optional<Pose2d> latestVisionPose = Optional.empty();
  
  // Timestamp of the latest vision measurement (in seconds)
  private double latestTimestamp = 0.0;
  
  // Number of AprilTags currently visible
  private int visibleTagCount = 0;
  
  // Distance to the hub (calculated from vision or odometry)
  private double distanceToHub = 0.0;
  
  // The target we're currently tracking (hub position)
  private Translation3d currentTargetPosition = RED_HUB_POSITION.getTranslation();
  
  // ============================================================================
  // CONSTRUCTOR
  // ============================================================================
  
  public VisionSubsystem() {
    // Initialize the Limelight
    configureLimelight();
  }
  
  /**
   * Configures the Limelight camera settings.
   * This sets up:
   * - Camera offset (where it's mounted on the robot)
   * - LED mode (when to turn on the LEDs)
   * - Pipeline (which vision processing mode to use)
   */
  private void configureLimelight() {
    // Tell the Limelight where the camera is mounted relative to the robot
    // This is CRITICAL for accurate pose estimation
    LimelightHelpers.setCameraPose_RobotSpace(
        LIMELIGHT_NAME,
        CAMERA_TO_ROBOT_OFFSET.getX(),
        CAMERA_TO_ROBOT_OFFSET.getY(),
        CAMERA_TO_ROBOT_OFFSET.getZ(),
        CAMERA_TO_ROBOT_OFFSET.getRotation().getX(),
        CAMERA_TO_ROBOT_OFFSET.getRotation().getY(),
        CAMERA_TO_ROBOT_OFFSET.getRotation().getZ()
    );
    
    // Set LED mode to "pipeline control" - the Limelight will decide when to use LEDs
    LimelightHelpers.setLEDMode_PipelineControl(LIMELIGHT_NAME);
    
    // Set to use pipeline 0 (AprilTag tracking)
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);
  }
  
  // ============================================================================
  // PERIODIC UPDATE
  // ============================================================================
  
  @Override
  public void periodic() {
    // This runs every 20ms (50 times per second)
    // It updates the robot's position estimate based on what the Limelight sees
    
    if (!visionEnabled) {
      return; // Skip if vision is disabled
    }
    
    // Check if we have a valid target
    // The LimelightHelpers API uses simple getter methods
    boolean hasTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);
    
    if (hasTarget) {
      
      // Get the robot's pose estimate from MegaTag2
      // This uses MULTIPLE AprilTags to calculate a more accurate position
      // The _wpiBlue suffix means it returns coordinates for blue alliance origin
      Pose2d robotPose = LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_NAME);
      
      // Get how many tags the Limelight can see
      // This comes from the botpose array - element 7 is the tag count
      double[] botposeArray = LimelightHelpers.getBotPose_wpiBlue(LIMELIGHT_NAME);
      if (botposeArray.length > 7) {
        visibleTagCount = (int) botposeArray[7]; // Index 7 is the tag count
      } else {
        visibleTagCount = 0;
      }
      
      // Only use the pose if we see enough tags (reduces bad estimates)
      if (visibleTagCount >= MIN_TAGS_FOR_POSE) {
        
        // Store the pose and timestamp
        latestVisionPose = Optional.of(robotPose);
        // Get current timestamp from Timer
        latestTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        
        // Calculate distance to the hub
        Translation2d hubPosition = currentTargetPosition.toTranslation2d();
        distanceToHub = robotPose.getTranslation().getDistance(hubPosition);
        
        // Log the data to AdvantageScope for visualization
        Logger.recordOutput("Vision/RobotPose", robotPose);
        Logger.recordOutput("Vision/VisibleTags", visibleTagCount);
        Logger.recordOutput("Vision/DistanceToHub", distanceToHub);
        Logger.recordOutput("Vision/HasValidTarget", true);
        
      } else {
        // Not enough tags visible - don't use this estimate
        Logger.recordOutput("Vision/HasValidTarget", false);
      }
      
    } else {
      // No valid target from Limelight
      latestVisionPose = Optional.empty();
      visibleTagCount = 0;
      Logger.recordOutput("Vision/HasValidTarget", false);
    }
  }
  
  // ============================================================================
  // POSE ESTIMATION
  // ============================================================================
  
  /**
   * Gets the latest vision estimate of the robot's position.
   * 
   * HOW TO USE THIS:
   * This should be called by the SwerveSubsystem to fuse vision data
   * with wheel odometry. The fusion creates a more accurate position estimate
   * than using either vision or wheels alone.
   * 
   * @return Optional containing the robot pose and timestamp, or empty if no valid estimate
   */
  public Optional<VisionMeasurement> getLatestVisionMeasurement() {
    if (latestVisionPose.isPresent()) {
      return Optional.of(new VisionMeasurement(
          latestVisionPose.get(),
          latestTimestamp,
          visibleTagCount
      ));
    }
    return Optional.empty();
  }
  
  /**
   * Simple container class to hold vision measurement data
   */
  public static class VisionMeasurement {
    public final Pose2d pose;           // Where the robot is
    public final double timestamp;      // When this measurement was taken
    public final int tagCount;          // How many tags were used
    
    public VisionMeasurement(Pose2d pose, double timestamp, int tagCount) {
      this.pose = pose;
      this.timestamp = timestamp;
      this.tagCount = tagCount;
    }
  }
  
  // ============================================================================
  // TARGET TRACKING
  // ============================================================================
  
  /**
   * Gets the position of a specific AprilTag target.
   * 
   * This is used for AIMING at targets like the hub.
   * Even if we're using odometry for our robot position, we can still
   * use vision to find where the hub actually is.
   * 
   * @param tagId The AprilTag ID to look for
   * @return The 3D position of the tag, or empty if not visible
   */
  public Optional<Pose3d> getTargetPose(int tagId) {
    // Check if we have any valid target
    boolean hasTarget = LimelightHelpers.getTV(LIMELIGHT_NAME);
    
    if (!hasTarget) {
      return Optional.empty();
    }
    
    // Get the fiducial ID of the currently tracked target
    double currentFiducialId = LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
    
    // Check if it matches the tag we're looking for
    if ((int) currentFiducialId == tagId) {
      // Get the 3D position of the target in robot space
      Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(LIMELIGHT_NAME);
      return Optional.of(targetPose);
    }
    
    return Optional.empty(); // Tag not visible or different tag in view
  }
  
  /**
   * Gets the position of the hub we should be aiming at.
   * 
   * SMART TARGETING:
   * 1. First tries to find the hub using its AprilTag
   * 2. If the tag isn't visible, falls back to the known hub position
   * 3. Returns the position in robot-relative coordinates for aiming
   * 
   * @param isRedAlliance Whether we're on the red alliance
   * @return The 3D position of the hub to aim at
   */
  public Translation3d getHubPosition(boolean isRedAlliance) {
    // Get the correct hub tag ID based on alliance
    int hubTagId = isRedAlliance ? RED_HUB_TAG_ID : BLUE_HUB_TAG_ID;
    
    // Try to get the hub position from vision
    Optional<Pose3d> visionHub = getTargetPose(hubTagId);
    
    if (visionHub.isPresent()) {
      // We can see the hub! Use its actual position
      Translation3d hubPos = visionHub.get().getTranslation();
      
      // Log that we're using vision
      Logger.recordOutput("Vision/UsingVisionForHub", true);
      
      // Update and return the position
      currentTargetPosition = hubPos;
      return hubPos;
      
    } else {
      // Can't see the hub tag, use the known position
      Logger.recordOutput("Vision/UsingVisionForHub", false);
      
      // Use the pre-programmed hub position
      Translation3d fallbackPos = isRedAlliance 
          ? RED_HUB_POSITION.getTranslation()
          : BLUE_HUB_POSITION.getTranslation();
      
      currentTargetPosition = fallbackPos;
      return fallbackPos;
    }
  }
  
  // ============================================================================
  // UTILITY METHODS
  // ============================================================================
  
  /**
   * Gets the distance from the robot to the hub.
   * 
   * @return Distance in meters
   */
  public double getDistanceToHub() {
    return distanceToHub;
  }
  
  /**
   * Checks if we have a valid vision target.
   * 
   * @return True if we can see enough AprilTags for a good estimate
   */
  public boolean hasValidTarget() {
    return visibleTagCount >= MIN_TAGS_FOR_POSE;
  }
  
  /**
   * Gets the number of AprilTags currently visible.
   * 
   * @return Number of visible tags
   */
  public int getVisibleTagCount() {
    return visibleTagCount;
  }
  
  /**
   * Enables or disables vision processing.
   * Useful for testing or if the Limelight fails.
   * 
   * @param enabled True to enable vision, false to disable
   */
  public void setVisionEnabled(boolean enabled) {
    this.visionEnabled = enabled;
    Logger.recordOutput("Vision/Enabled", enabled);
  }
  
  /**
   * Checks if vision is currently enabled.
   * 
   * @return True if vision is enabled
   */
  public boolean isVisionEnabled() {
    return visionEnabled;
  }
  
  /**
   * Toggles the Limelight LEDs on/off.
   * Useful for demos or conserving battery.
   * 
   * @param on True to turn LEDs on, false to turn off
   */
  public void setLEDsOn(boolean on) {
    if (on) {
      LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_NAME);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_NAME);
    }
  }
}
