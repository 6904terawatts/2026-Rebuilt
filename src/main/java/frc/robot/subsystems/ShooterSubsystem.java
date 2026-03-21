package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  
  // ============================================================================
  // KRAKEN X60 MOTOR (Direct Phoenix 6 control)
  // ============================================================================
  
  private final TalonFX shooterKraken;
  private final VelocityVoltage velocityRequest;
  private double targetVelocityRPS;

  public ShooterSubsystem() {
    shooterKraken = new TalonFX(Constants.ShooterConstants.kLeaderMotorId);
    velocityRequest = new VelocityVoltage(0);
    targetVelocityRPS = 0;
    
    configureKrakenMotor();
  }

  /**
   * Configure Kraken X60 shooter motor
   */
  private void configureKrakenMotor() {
    try {
      System.out.println("🔧 Configuring Kraken X60 shooter motor...");
      
      TalonFXConfiguration config = new TalonFXConfiguration();
      
      // Current limits - Kraken can handle more power for shooter!
      config.CurrentLimits.StatorCurrentLimit = 40;  // Motor current (was 35A for NEO)
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = 30;  // Battery current
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      
      // Motor output
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;  // Coast for shooter
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  // Change if backwards
      
      // Voltage compensation
      config.Voltage.PeakForwardVoltage = 12.0;
      config.Voltage.PeakReverseVoltage = -12.0;
      
      // PID configuration for velocity control
      Slot0Configs slot0 = new Slot0Configs();
      slot0.kP = 0.15;    // Start here, tune as needed
      slot0.kI = 0.0;    // Usually not needed for velocity
      slot0.kD = 0.0;    // Usually not needed for velocity
      slot0.kV = 0.12;   // Feedforward (12V / 100 RPS = 0.12) - CRITICAL!
      config.Slot0 = slot0;
      
      // Apply configuration
      shooterKraken.getConfigurator().apply(config);
      
      System.out.println("  ↳ Stator limit: 60A");
      System.out.println("  ↳ Supply limit: 40A");
      System.out.println("  ↳ Coast mode");
      System.out.println("  ↳ PID: P=0.1, V=0.12");
      System.out.println("✅ Kraken X60 shooter configured");
      
    } catch (Exception e) {
      System.err.println("⚠️ Error configuring Kraken: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /**
   * Set shooter speed
   * @param speed Angular velocity (will be converted from RPM to RPS)
   */
  public Command setSpeed(AngularVelocity speed) {
    return Commands.run(() -> {
      // Convert RPM to RPS for Kraken (Kraken uses RPS not RPM!)
      double rps = speed.in(RPM) / 60.0;
      targetVelocityRPS = rps;
      shooterKraken.setControl(velocityRequest.withVelocity(rps));
    }, this).withName("Shooter.SetSpeed");
  }

  /**
   * Set shooter speed dynamically from supplier
   */
  public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
    return Commands.run(() -> {
      AngularVelocity speed = speedSupplier.get();
      double rps = speed.in(RPM) / 60.0;
      targetVelocityRPS = rps;
      shooterKraken.setControl(velocityRequest.withVelocity(rps));
    }, this).withName("Shooter.SetSpeedDynamic");
  }

  /**
   * Spin up shooter to target speed (5800 RPM)
   */
  public Command spinUp() {
    return Commands.run(() -> {
      // 5800 RPM = 96.67 RPS
      double targetRPS = 3500.0 / 60.0;
      targetVelocityRPS = targetRPS;
      shooterKraken.setControl(velocityRequest.withVelocity(targetRPS));
    }, this).withName("Shooter.SpinUp");
  }

  /**
   * Stop shooter
   */
  public Command stop() {
    return Commands.runOnce(() -> {
      targetVelocityRPS = 0;
      shooterKraken.setControl(velocityRequest.withVelocity(0));
    }, this).withName("Shooter.Stop");
  }

  /**
   * Get current shooter speed as AngularVelocity
   */
  public AngularVelocity getSpeed() {
    // Get current velocity in RPS from Kraken
    double currentRPS = shooterKraken.getVelocity().getValueAsDouble();
    // Convert RPS to RPM and return
    return RPM.of(currentRPS * 60.0);
  }

  /**
   * Check if shooter is at target speed
   */
  public boolean atTargetSpeed() {
    double currentVelocity = shooterKraken.getVelocity().getValueAsDouble();
    return Math.abs(currentVelocity - targetVelocityRPS) < 2.0;  // Within 2 RPS
  }

  /**
   * Get current velocity in RPM
   */
  public double getVelocityRPM() {
    return shooterKraken.getVelocity().getValueAsDouble() * 60.0;  // RPS to RPM
  }

  /**
   * SysID characterization command
   */
  public Command sysId() {
    // Note: You may need to implement custom SysID for Kraken
    // This is a placeholder - adjust based on your SysID needs
    return Commands.none();
  }

  @Override
  public void periodic() {
    // SmartDashboard telemetry
    
    
    // AdvantageKit logging
    Logger.recordOutput("Shooter/KrakenVelocity", shooterKraken.getVelocity().getValue());
    Logger.recordOutput("Shooter/KrakenCurrent", shooterKraken.getSupplyCurrent().getValue());
    Logger.recordOutput("Shooter/KrakenTemp", shooterKraken.getDeviceTemp().getValue());
    Logger.recordOutput("Shooter/TargetVelocityRPS", targetVelocityRPS);
    Logger.recordOutput("Shooter/AtSpeed", atTargetSpeed());
  }

  @Override
  public void simulationPeriodic() {
    // Kraken simulation can be added here if needed
    // Phoenix 6 has built-in sim support
  }

  /**
   * Get wheel radius
   */
  private Distance wheelRadius() {
    return Inches.of(4).div(2);  // 4 inch diameter wheels
  }

  /**
   * Get tangential velocity at edge of shooter wheel
   */
  public LinearVelocity getTangentialVelocity() {
    // Get velocity in RPS from Kraken
    double rps = shooterKraken.getVelocity().getValueAsDouble();
    // Convert RPS to radians per second
    double radPerSec = rps * 2.0 * Math.PI;
    // Calculate tangential velocity
    return MetersPerSecond.of(radPerSec * wheelRadius().in(Meters));
  }
}