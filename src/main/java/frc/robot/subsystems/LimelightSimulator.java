package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

/**
 * Simulates Limelight data for testing auto-align in AdvantageScope
 * 
 * ONLY ACTIVE IN SIMULATION - does nothing on real robot
 */
public class LimelightSimulator extends SubsystemBase {
  
  private final NetworkTable limelightTable;
  
  // Simulated values
  private boolean hasTarget = false;
  private double tx = 0;  // Horizontal offset (-29.8 to 29.8 degrees)
  private double ty = 0;  // Vertical offset
  private double ta = 0;  // Target area (0-100%)
  private int tagId = 27; // Default to blue center tag
  
  public LimelightSimulator() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    System.out.println("🎮 Limelight Simulator created (sim only)");
  }
  
  /**
   * Simulate seeing a hub tag
   */
  public void simulateHubTag(boolean visible, double horizontalOffset, double area, int aprilTagId) {
    hasTarget = visible;
    tx = horizontalOffset;
    ta = area;
    tagId = aprilTagId;
    
    System.out.println("🎯 Limelight Sim: " + 
        (visible ? "Tag " + aprilTagId + " @ TX=" + tx + "°, TA=" + ta + "%" : "No target"));
  }
  
  /**
   * PRESET 1: Robot sees center tag, slightly off-center, medium distance
   */
  public void simulateDefaultView() {
    int centerTag = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
        == DriverStation.Alliance.Blue ? 27 : 8;
    simulateHubTag(true, 5.0, 2.5, centerTag);  // 5° left, 2.5% area (medium distance)
  }
  
  /**
   * PRESET 2: Robot perfectly aligned
   */
  public void simulateAligned() {
    int centerTag = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
        == DriverStation.Alliance.Blue ? 27 : 8;
    simulateHubTag(true, 0.0, 3.0, centerTag);  // Centered, correct distance
  }
  
  /**
   * PRESET 3: Robot too close
   */
  public void simulateTooClose() {
    int centerTag = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
        == DriverStation.Alliance.Blue ? 27 : 8;
    simulateHubTag(true, 0.0, 5.0, centerTag);  // Centered, too close (>3.5% area)
  }
  
  /**
   * PRESET 4: Robot too far
   */
  public void simulateTooFar() {
    int centerTag = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
        == DriverStation.Alliance.Blue ? 27 : 8;
    simulateHubTag(true, 0.0, 1.0, centerTag);  // Centered, too far (<2% area)
  }
  
  /**
   * PRESET 5: Robot way off to the left
   */
  public void simulateOffLeft() {
    int centerTag = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
        == DriverStation.Alliance.Blue ? 27 : 8;
    simulateHubTag(true, 15.0, 2.5, centerTag);  // 15° left, medium distance
  }
  
  /**
   * PRESET 6: Robot way off to the right
   */
  public void simulateOffRight() {
    int centerTag = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) 
        == DriverStation.Alliance.Blue ? 27 : 8;
    simulateHubTag(true, -15.0, 2.5, centerTag);  // 15° right, medium distance
  }
  
  /**
   * PRESET 7: No target visible
   */
  public void simulateNoTarget() {
    simulateHubTag(false, 0, 0, 0);
  }
  
  @Override
  public void periodic() {
    // ONLY publish NetworkTables data in simulation
    if (!Robot.isReal()) {
      limelightTable.getEntry("tv").setDouble(hasTarget ? 1 : 0);
      limelightTable.getEntry("tx").setDouble(tx);
      limelightTable.getEntry("ty").setDouble(ty);
      limelightTable.getEntry("ta").setDouble(ta);
      limelightTable.getEntry("tid").setDouble(tagId);
      
      // Extra telemetry for AdvantageScope
      SmartDashboard.putBoolean("LimelightSim/Has Target", hasTarget);
      SmartDashboard.putNumber("LimelightSim/TX", tx);
      SmartDashboard.putNumber("LimelightSim/TA", ta);
      SmartDashboard.putNumber("LimelightSim/Tag ID", tagId);
    }
  }
}