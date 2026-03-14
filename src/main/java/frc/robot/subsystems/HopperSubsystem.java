package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {

  private static final double HOPPER_SPEED = 1.00;

  private TalonFX hopperKraken = new TalonFX(Constants.HopperConstants.kHopperMotorId);
  private DutyCycleOut hopperDutyCycleRequest = new DutyCycleOut(0);

  public HopperSubsystem() {
    configureKrakenMotor();
  }

  private void configureKrakenMotor() {
    try {
      System.out.println("🔧 Configuring Kraken X60 hopper motor...");
      
      TalonFXConfiguration config = new TalonFXConfiguration();
      
      // Current limits - Kraken can handle more than NEO
      config.CurrentLimits.StatorCurrentLimit = 40;  // Motor current (was 35A for NEO)
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = 30;  // Battery current
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      
      // Motor output
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      //config.MotorOutput.Inverted = true;  // Match your current setup, flip if backwards
      
      // Voltage compensation
      config.Voltage.PeakForwardVoltage = 12.0;
      config.Voltage.PeakReverseVoltage = -12.0;
      
      // Ramp rate for smooth acceleration
      config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
      
      // Apply configuration
      hopperKraken.getConfigurator().apply(config);
      
      System.out.println("  ↳ Stator limit: 40A");
      System.out.println("  ↳ Supply limit: 30A");
      System.out.println("  ↳ Brake mode enabled");
      System.out.println("✅ Kraken X60 hopper motor configured");
      
    } catch (Exception e) {
      System.err.println("⚠️ Error configuring Kraken: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /**
   * Command to run the hopper forward while held.
   */
  public Command feedCommand() {
    return Commands.run(() -> {
      hopperKraken.setControl(hopperDutyCycleRequest.withOutput(-HOPPER_SPEED));
    }, this)
    .finallyDo(() -> {
      hopperKraken.setControl(hopperDutyCycleRequest.withOutput(0));
    })
    .withName("Hopper.Feed");
  }

  public Command backFeedCommand() {
    return Commands.run(() -> {
      hopperKraken.setControl(hopperDutyCycleRequest.withOutput(HOPPER_SPEED));
    }, this)
    .finallyDo(() -> {
      hopperKraken.setControl(hopperDutyCycleRequest.withOutput(0));
    })
    .withName("Hopper.BackFeed");
  }

  /**
   * Command to run the hopper in reverse while held.
   */
  public Command reverseCommand() {
    return Commands.run(() -> {
      hopperKraken.setControl(hopperDutyCycleRequest.withOutput(HOPPER_SPEED));
    }, this)
    .finallyDo(() -> {
      hopperKraken.setControl(hopperDutyCycleRequest.withOutput(0));
    })
    .withName("Hopper.Reverse");
  }

  /**
   * Command to stop the hopper.
   */
  public Command stopCommand() {
    return Commands.runOnce(() -> {
      hopperKraken.setControl(hopperDutyCycleRequest.withOutput(0));
    }, this)
    .withName("Hopper.Stop");
  }

  @Override
  public void periodic() {
    // Telemetry
    // SmartDashboard.putNumber("Hopper/Current", hopperKraken.getSupplyCurrent().getValue());
    // SmartDashboard.putNumber("Hopper/Voltage", hopperKraken.getMotorVoltage().getValue());
    // SmartDashboard.putNumber("Hopper/Temp", hopperKraken.getDeviceTemp().getValue());
    // SmartDashboard.putNumber("Hopper/StatorCurrent", hopperKraken.getStatorCurrent().getValue());
  }

  @Override
  public void simulationPeriodic() {
    // Simulation support can be added here if needed
  }
}