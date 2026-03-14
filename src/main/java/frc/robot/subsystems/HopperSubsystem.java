package frc.robot.subsystems;
 
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
 
// KRAKEN IMPORTS (Uncomment when switching to Kraken):
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.controls.DutyCycleOut;
 
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
// KRAKEN: import yams.motorcontrollers.local.TalonFXWrapper;
 
public class HopperSubsystem extends SubsystemBase {
 
  private static final double HOPPER_SPEED = 1.00;
 
  // ============================================================================
  // HOPPER MOTOR - CURRENT: NEO Vortex with SparkFlex
  // ============================================================================
  
  // NEO VORTEX VERSION (Current):
  private SparkFlex hopperSpark = new SparkFlex(Constants.HopperConstants.kHopperMotorId, MotorType.kBrushless);
 
  // KRAKEN X60 VERSION (Uncomment when switching):
  // private TalonFX hopperKraken = new TalonFX(Constants.HopperConstants.kHopperMotorId);
  // private DutyCycleOut hopperDutyCycleRequest = new DutyCycleOut(0);
 
  // ============================================================================
  // MOTOR CONFIGURATION
  // ============================================================================
  
  // NEO VORTEX CONFIG (Current):
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) // 4:1 gear reduction
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(35));   
 
  private SmartMotorController smc = new SparkWrapper(hopperSpark, DCMotor.getNeoVortex(1), smcConfig);
 
  // KRAKEN X60 CONFIG (Uncomment when switching):
  // private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  //     .withControlMode(ControlMode.OPEN_LOOP)
  //     .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
  //     .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
  //     .withMotorInverted(true)
  //     .withIdleMode(MotorMode.BRAKE)
  //     .withStatorCurrentLimit(Amps.of(40));  // Kraken can handle more
 
  // private SmartMotorController smc = new TalonFXWrapper(hopperKraken, DCMotor.getKrakenX60(1), smcConfig);
 
  private final FlyWheelConfig hopperConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      // .withUpperSoftLimit(RPM.of(6000))
      // .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Hopper", TelemetryVerbosity.HIGH);
 
  private FlyWheel hopper = new FlyWheel(hopperConfig);
 
  public HopperSubsystem() {
    // ============================================================================
    // KRAKEN MOTOR CONFIGURATION (Uncomment when switching)
    // ============================================================================
    
    // configureKrakenMotor();
  }
 
  // ============================================================================
  // KRAKEN CONFIGURATION METHOD (Uncomment when switching)
  // ============================================================================
  
  // private void configureKrakenMotor() {
  //   try {
  //     System.out.println("🔧 Configuring Kraken X60 hopper motor...");
  //     
  //     TalonFXConfiguration config = new TalonFXConfiguration();
  //     
  //     // Current limits - Kraken can handle more than NEO
  //     config.CurrentLimits.StatorCurrentLimit = 40;  // Motor current (was 35A for NEO)
  //     config.CurrentLimits.StatorCurrentLimitEnable = true;
  //     config.CurrentLimits.SupplyCurrentLimit = 30;  // Battery current
  //     config.CurrentLimits.SupplyCurrentLimitEnable = true;
  //     
  //     // Motor output
  //     config.MotorOutput.NeutralMode = NeutralModeValue.Brake;  // Match current brake mode
  //     config.MotorOutput.Inverted = true;  // Match current inversion
  //     
  //     // Voltage compensation (keeps performance consistent as battery drains)
  //     config.Voltage.PeakForwardVoltage = 12.0;
  //     config.Voltage.PeakReverseVoltage = -12.0;
  //     
  //     // Optional: Ramp rate for smooth acceleration
  //     config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;  // 0.1 sec to full speed
  //     
  //     // Apply configuration to motor
  //     hopperKraken.getConfigurator().apply(config);
  //     
  //     System.out.println("  ↳ Kraken motor configured");
  //     System.out.println("  ↳ Stator limit: 40A (motor)");
  //     System.out.println("  ↳ Supply limit: 30A (battery)");
  //     System.out.println("  ↳ Neutral mode: Brake");
  //     System.out.println("  ↳ Inverted: true");
  //     System.out.println("✅ Kraken X60 hopper motor ready");
  //     
  //   } catch (Exception e) {
  //     System.err.println("⚠️ Error configuring Kraken: " + e.getMessage());
  //     e.printStackTrace();
  //   }
  // }
 
  // ============================================================================
  // HOPPER COMMANDS
  // ============================================================================
 
  /**
   * Command to run the hopper forward while held.
   */
  public Command feedCommand() {
    return hopper.set(-HOPPER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Hopper.Feed");
    
    // KRAKEN VERSION (Uncomment when switching):
    // return Commands.run(() -> {
    //   hopperKraken.setControl(hopperDutyCycleRequest.withOutput(-HOPPER_SPEED));
    // })
    // .finallyDo(() -> {
    //   hopperKraken.setControl(hopperDutyCycleRequest.withOutput(0));
    // })
    // .withName("Hopper.Feed");
  }
 
  public Command backFeedCommand() {
    return hopper.set(HOPPER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Hopper.BackFeed");
    
    // KRAKEN VERSION (Uncomment when switching):
    // return Commands.run(() -> {
    //   hopperKraken.setControl(hopperDutyCycleRequest.withOutput(HOPPER_SPEED));
    // })
    // .finallyDo(() -> {
    //   hopperKraken.setControl(hopperDutyCycleRequest.withOutput(0));
    // })
    // .withName("Hopper.BackFeed");
  }
 
  /**
   * Command to run the hopper in reverse while held.
   */
  public Command reverseCommand() {
    return hopper.set(HOPPER_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Hopper.Reverse");
    
    // KRAKEN VERSION (Uncomment when switching):
    // return Commands.run(() -> {
    //   hopperKraken.setControl(hopperDutyCycleRequest.withOutput(HOPPER_SPEED));
    // })
    // .finallyDo(() -> {
    //   hopperKraken.setControl(hopperDutyCycleRequest.withOutput(0));
    // })
    // .withName("Hopper.Reverse");
  }
 
  /**
   * Command to stop the hopper.
   */
  public Command stopCommand() {
    return hopper.set(0).withName("Hopper.Stop");
    
    // KRAKEN VERSION (Uncomment when switching):
    // return Commands.runOnce(() -> {
    //   hopperKraken.setControl(hopperDutyCycleRequest.withOutput(0));
    // })
    // .withName("Hopper.Stop");
  }
 
  @Override
  public void periodic() {
    hopper.updateTelemetry();
    
    // KRAKEN TELEMETRY (Uncomment when switching):
    // SmartDashboard.putNumber("Hopper/Current", hopperKraken.getSupplyCurrent().getValue());
    // SmartDashboard.putNumber("Hopper/Voltage", hopperKraken.getMotorVoltage().getValue());
    // SmartDashboard.putNumber("Hopper/Temp", hopperKraken.getDeviceTemp().getValue());
    // SmartDashboard.putNumber("Hopper/StatorCurrent", hopperKraken.getStatorCurrent().getValue());
  }
 
  @Override
  public void simulationPeriodic() {
    hopper.simIterate();
  }
}