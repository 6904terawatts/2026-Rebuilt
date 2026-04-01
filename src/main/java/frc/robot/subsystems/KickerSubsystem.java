package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
  
  // ============================================================================
  // DUAL NEO MOTORS (Leader + Follower Inverted)
  // ============================================================================
  
  private static final double KICKER_SPEED = 0.8;  // 80% duty cycle
  
  private final SparkMax leaderKicker;
  private final SparkMax followerKicker;

  public KickerSubsystem() {
    leaderKicker = new SparkMax(Constants.KickerConstants.kLeaderKickerMotorId, MotorType.kBrushless);
    followerKicker = new SparkMax(Constants.KickerConstants.kFollowerKickerMotorId, MotorType.kBrushless);
    
    configureMotors();
  }

  /**
   * Configure both kicker motors
   */
  @SuppressWarnings("removal")
  private void configureMotors() {
    try {
      System.out.println("🔧 Configuring dual NEO kicker motors...");
      
      // ====================================================================
      // LEADER MOTOR CONFIGURATION
      // ====================================================================
      
      SparkMaxConfig leaderConfig = new SparkMaxConfig();
      
      // Current limits - 40A max
      leaderConfig.smartCurrentLimit(40);
      
      // Idle mode - brake to stop quickly
      leaderConfig.idleMode(IdleMode.kBrake);
      
      // Inversion - normal direction
      leaderConfig.inverted(true);
      
      // Voltage compensation
      leaderConfig.voltageCompensation(12.0);
      
      // Apply leader config
      leaderKicker.configure(leaderConfig, SparkMax.ResetMode.kResetSafeParameters, 
          SparkMax.PersistMode.kPersistParameters);
      
      // ====================================================================
      // FOLLOWER MOTOR CONFIGURATION
      // ====================================================================
      
      SparkMaxConfig followerConfig = new SparkMaxConfig();
      
      // Current limits - 40A max
      followerConfig.smartCurrentLimit(40);
      
      // Idle mode - brake
      followerConfig.idleMode(IdleMode.kBrake);
      
      // INVERTED from leader (opposite side of kicker roller)
      followerConfig.inverted(true);
      
      // Voltage compensation
      followerConfig.voltageCompensation(12.0);
      
      // Set follower to follow leader
      //followerConfig.follow(leaderKicker);
      
      // Apply follower config
      followerKicker.configure(followerConfig, SparkMax.ResetMode.kResetSafeParameters, 
          SparkMax.PersistMode.kPersistParameters);
      
      System.out.println("  ↳ Leader: CAN " + Constants.KickerConstants.kLeaderKickerMotorId);
      System.out.println("  ↳ Follower: CAN " + Constants.KickerConstants.kFollowerKickerMotorId + " (inverted)");
      System.out.println("  ↳ Current limit: 40A per motor");
      System.out.println("  ↳ Brake mode");
      System.out.println("  ↳ Kicker speed: " + (KICKER_SPEED * 100) + "%");
      System.out.println("✅ Dual NEO kicker configured");
      
    } catch (Exception e) {
      System.err.println("⚠️ Error configuring kicker: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /**
   * Feed game pieces through kicker (forward)
   */
  public Command feedCommand() {
    return Commands.run(() -> {
      leaderKicker.set(KICKER_SPEED);
      followerKicker.set(-KICKER_SPEED);
      // Follower automatically follows
      System.out.println("🎯 KICKER FEEDING!");
    }, this).withName("Kicker.Feed");
  }

  /**
   * Reverse kicker (unjam/eject)
   */
  public Command reverseCommand() {
    return Commands.run(() -> {
      leaderKicker.set(-KICKER_SPEED);
      followerKicker.set(KICKER_SPEED);
      // Follower automatically follows (inverted)
      System.out.println("🔄 KICKER REVERSING!");
    }, this).withName("Kicker.Reverse");
  }

  /**
   * Stop kicker
   */
  public Command stopCommand() {
    return Commands.runOnce(() -> {
      leaderKicker.set(0);
      followerKicker.set(0);
      // Follower automatically stops
      System.out.println("🛑 KICKER STOPPED");
    }, this).withName("Kicker.Stop");
  }

  @Override
  public void periodic() {
    // SmartDashboard telemetry
    SmartDashboard.putNumber("Kicker/Leader Current", leaderKicker.getOutputCurrent());
    SmartDashboard.putNumber("Kicker/Follower Current", followerKicker.getOutputCurrent());
    SmartDashboard.putNumber("Kicker/Total Current", 
        leaderKicker.getOutputCurrent() + followerKicker.getOutputCurrent());
    SmartDashboard.putNumber("Kicker/Leader Velocity", leaderKicker.getEncoder().getVelocity());
    SmartDashboard.putNumber("Kicker/Follower Velocity", followerKicker.getEncoder().getVelocity());
    SmartDashboard.putNumber("Kicker/Leader Temp", leaderKicker.getMotorTemperature());
    SmartDashboard.putNumber("Kicker/Follower Temp", followerKicker.getMotorTemperature());
    
    // AdvantageKit logging
    Logger.recordOutput("Kicker/LeaderCurrent", leaderKicker.getOutputCurrent());
    Logger.recordOutput("Kicker/FollowerCurrent", followerKicker.getOutputCurrent());
    Logger.recordOutput("Kicker/LeaderVelocity", leaderKicker.getEncoder().getVelocity());
    Logger.recordOutput("Kicker/FollowerVelocity", followerKicker.getEncoder().getVelocity());
  }
}