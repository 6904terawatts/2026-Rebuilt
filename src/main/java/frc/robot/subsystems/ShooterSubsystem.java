package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

// ============================================================================
// KRAKEN IMPORTS - Uncomment when switching to Kraken X60:
// ============================================================================
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class ShooterSubsystem extends SubsystemBase {
  
  // ============================================================================
  // NEO VORTEX VERSION (Current - Using YAMS)
  // ============================================================================
  
  // 2 Neos, 4in shooter wheels
  // private final ThriftyNova leaderNova = new ThriftyNova(
  // Constants.ShooterConstants.kLeaderMotorId,
  // ThriftyNova.MotorType.NEO);

  // private final ThriftyNova followerNova = new ThriftyNova(
  // Constants.ShooterConstants.kFollowerMotorId,
  // ThriftyNova.MotorType.NEO);

  private final SparkFlex leaderSpark = new SparkFlex(Constants.ShooterConstants.kLeaderMotorId,
      MotorType.kBrushless);

  // private final SparkFlex followerSpark = new SparkFlex(Constants.ShooterConstants.kFollowerMotorId,
  //     MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      //.withFollowers(Pair.of(followerSpark, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.00936, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(35));

  private final SmartMotorController smc = new SparkWrapper(leaderSpark, DCMotor.getNeoVortex(1), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(0))
      .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  private final FlyWheel shooter = new FlyWheel(shooterConfig);

  // ============================================================================
  // KRAKEN X60 VERSION - Uncomment when switching (NO YAMS wrapper!)
  // ============================================================================
  
  // KRAKEN MOTOR DECLARATION:
  // private final TalonFX shooterKraken = new TalonFX(Constants.ShooterConstants.kLeaderMotorId);
  // private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  // private double targetVelocityRPS = 0;

  public ShooterSubsystem() {
    // leaderNova.factoryReset();
    // followerNova.factoryReset();

    // leaderNova.setVoltageCompensation(12);
    // followerNova.setVoltageCompensation(12);

    // leaderNova.setInverted(false);
    // followerNova.setInverted(true);

    // followerNova
    // .setInversion(true)
    // .follow(leaderNova.getID());
    
    // ============================================================
    // KRAKEN - Uncomment when switching:
    // ============================================================
    // configureKrakenMotor();
  }

  // ============================================================================
  // KRAKEN CONFIGURATION METHOD - Uncomment when switching
  // ============================================================================
  
  // private void configureKrakenMotor() {
  //   try {
  //     System.out.println("🔧 Configuring Kraken X60 shooter...");
  //     
  //     TalonFXConfiguration config = new TalonFXConfiguration();
  //     
  //     // Current limits - Kraken can handle more!
  //     config.CurrentLimits.StatorCurrentLimit = 60;  // Motor current (was 35A)
  //     config.CurrentLimits.StatorCurrentLimitEnable = true;
  //     config.CurrentLimits.SupplyCurrentLimit = 40;  // Battery current
  //     config.CurrentLimits.SupplyCurrentLimitEnable = true;
  //     
  //     // Motor output
  //     config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  //     config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  // Change if backwards
  //     
  //     // Voltage
  //     config.Voltage.PeakForwardVoltage = 12.0;
  //     config.Voltage.PeakReverseVoltage = -12.0;
  //     
  //     // PID for velocity control
  //     Slot0Configs slot0 = new Slot0Configs();
  //     slot0.kP = 0.1;    // Start here, tune as needed
  //     slot0.kI = 0.0;
  //     slot0.kD = 0.0;
  //     slot0.kV = 0.12;   // Feedforward - CRITICAL! (12V / 100 RPS)
  //     config.Slot0 = slot0;
  //     
  //     shooterKraken.getConfigurator().apply(config);
  //     
  //     System.out.println("✅ Kraken configured: 60A, Coast, PID: P=0.1 V=0.12");
  //   } catch (Exception e) {
  //     System.err.println("⚠️ Kraken config error: " + e.getMessage());
  //   }
  // }

  // ============================================================================
  // COMMANDS
  // ============================================================================

  public Command setSpeed(AngularVelocity speed) {
    return shooter.setSpeed(speed);
    
    // KRAKEN VERSION - Uncomment when switching:
    // return Commands.run(() -> {
    //   double rps = speed.in(RPM) / 60.0;  // Convert RPM to RPS!
    //   targetVelocityRPS = rps;
    //   shooterKraken.setControl(velocityRequest.withVelocity(rps));
    // }, this);
  }

  public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
    return shooter.setSpeed(speedSupplier);
    
    // KRAKEN VERSION - Uncomment when switching:
    // return Commands.run(() -> {
    //   AngularVelocity speed = speedSupplier.get();
    //   double rps = speed.in(RPM) / 60.0;  // Convert RPM to RPS!
    //   targetVelocityRPS = rps;
    //   shooterKraken.setControl(velocityRequest.withVelocity(rps));
    // }, this);
  }

  public Command spinUp() {
    return Commands.run(() -> {
        shooter.setSpeed(RPM.of(5800)).schedule();
    }, this).withName("Shooter.SpinUp"); 
    
    // KRAKEN VERSION - Uncomment when switching:
    // return Commands.run(() -> {
    //   double targetRPS = 5800.0 / 60.0;  // 5800 RPM = 96.67 RPS
    //   targetVelocityRPS = targetRPS;
    //   shooterKraken.setControl(velocityRequest.withVelocity(targetRPS));
    // }, this).withName("Shooter.SpinUp");

    // return setSpeed(RotationsPerSecond.of(50));

    // return run(() -> {
    // // followerNova.follow(leaderNova.getID());
    // // followerNova.setInverted(true);

    // // leaderNova.setPercent(SHOOTER_SPEED);
    // // followerNova.setPercent(SHOOTER_SPEED);

    // // followerNova.setPercent(0.5);
    // });

    // return shooter.set(0.5);
    // return shooter.setSpeed(RotationsPerSecond.of(500));
  }

  public Command stop() {
    return Commands.run(() -> {
        shooter.setSpeed(RPM.of(0)).schedule();
    }, this).withName("Shooter.Stop"); 
    
    // KRAKEN VERSION - Uncomment when switching:
    // return Commands.runOnce(() -> {
    //   targetVelocityRPS = 0;
    //   shooterKraken.setControl(velocityRequest.withVelocity(0));
    // }, this).withName("Shooter.Stop");

    // // leaderNova.setPercent(0);
    // // followerNova.setPercent(0);
    // // followerNova.setPercent(0.5);
    // });
    // return shooter.set(0);
  }

  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
    
    // KRAKEN VERSION - Uncomment when switching:
    // double currentRPS = shooterKraken.getVelocity().getValue();
    // return RPM.of(currentRPS * 60.0);  // Convert RPS to RPM
  }

  // public Command set(double dutyCycle) {
  // return shooter.set(dutyCycle);
  // }

  public Command sysId() {
    return shooter.sysId(Volts.of(12), Volts.of(3).per(Second), Seconds.of(7));
  }
  
  // ============================================================================
  // KRAKEN UTILITY METHODS - Uncomment when switching
  // ============================================================================
  
  // public boolean atTargetSpeed() {
  //   double currentVelocity = shooterKraken.getVelocity().getValue();
  //   return Math.abs(currentVelocity - targetVelocityRPS) < 2.0;  // Within 2 RPS
  // }
  
  // public double getVelocityRPM() {
  //   return shooterKraken.getVelocity().getValue() * 60.0;  // RPS to RPM
  // }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/LeaderVelocity", leaderSpark.getEncoder().getVelocity());
    //Logger.recordOutput("Shooter/FollowerVelocity", followerSpark.getEncoder().getVelocity());
    
    // KRAKEN TELEMETRY - Uncomment when switching:
    // SmartDashboard.putNumber("Shooter/Target RPM", targetVelocityRPS * 60.0);
    // SmartDashboard.putNumber("Shooter/Current RPM", getVelocityRPM());
    // SmartDashboard.putNumber("Shooter/Current", shooterKraken.getSupplyCurrent().getValue());
    // SmartDashboard.putNumber("Shooter/Voltage", shooterKraken.getMotorVoltage().getValue());
    // SmartDashboard.putNumber("Shooter/Temp", shooterKraken.getDeviceTemp().getValue());
    // SmartDashboard.putBoolean("Shooter/At Speed", atTargetSpeed());
    // Logger.recordOutput("Shooter/KrakenVelocity", shooterKraken.getVelocity().getValue());
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  private Distance wheelRadius() {
    return Inches.of(4).div(2);
  }

  public LinearVelocity getTangentialVelocity() {
    // Calculate tangential velocity at the edge of the wheel and convert to
    // LinearVelocity

    return MetersPerSecond.of(getSpeed().in(RadiansPerSecond)
        * wheelRadius().in(Meters));
    
    // KRAKEN VERSION - Uncomment when switching:
    // double rps = shooterKraken.getVelocity().getValue();
    // double radPerSec = rps * 2.0 * Math.PI;  // RPS to rad/s
    // return MetersPerSecond.of(radPerSec * wheelRadius().in(Meters));
  }
}

// ================================================================================
// 🔄 KRAKEN X60 CONVERSION INSTRUCTIONS
// ================================================================================
//
// TO SWITCH FROM NEO VORTEX TO KRAKEN X60:
//
// 1. HARDWARE:
//    ✅ Install Kraken X60 motor
//    ✅ Connect to CAN bus (use same CAN ID)
//    ✅ Update firmware via Phoenix Tuner X
//
// 2. CODE CHANGES:
//    ✅ Uncomment KRAKEN IMPORTS at top of file
//    ✅ Comment out: NEO SparkFlex, YAMS stuff (lines 48-62)
//    ✅ Uncomment: Kraken motor declarations (lines 68-70)
//    ✅ Uncomment: configureKrakenMotor() call in constructor (line 87)
//    ✅ Uncomment: configureKrakenMotor() method (lines 93-120)
//    ✅ In EACH command, comment NEO version, uncomment KRAKEN version:
//       - setSpeed() (lines 126-133)
//       - setSpeedDynamic() (lines 135-142)
//       - spinUp() (lines 144-151)
//       - stop() (lines 169-174)
//       - getSpeed() (lines 176-179)
//    ✅ Uncomment utility methods (lines 191-198)
//    ✅ Uncomment telemetry in periodic() (lines 203-210)
//    ✅ Uncomment Kraken version of getTangentialVelocity() (lines 222-225)
//
// 3. TEST:
//    ✅ Deploy code
//    ✅ Open Phoenix Tuner X - verify motor appears
//    ✅ Test spinUp command
//    ✅ Check SmartDashboard values
//    ✅ Tune PID if needed (adjust kP and kV in configureKrakenMotor)
//    ✅ If backwards, change: InvertedValue.CounterClockwise_Positive
//
// ================================================================================
// ⚠️ CRITICAL NOTES
// ================================================================================
//
// UNITS CONVERSION (VERY IMPORTANT!):
//   NEO (YAMS): Uses RPM
//   KRAKEN (Phoenix 6): Uses RPS (rotations per second)
//   Conversion: RPS = RPM / 60.0
//   Example: 5800 RPM = 96.67 RPS
//
// PID TUNING:
//   kP = 0.1   (start here, tune as needed)
//   kV = 0.12  (CRITICAL for velocity! Feedforward = 12V / 100 RPS)
//   - Increase kP if too slow to reach target
//   - Decrease kP if oscillating
//
// CURRENT LIMITS:
//   Stator: 60A (motor) - Kraken can handle more than NEO!
//   Supply: 40A (battery)
//
// KRAKEN ADVANTAGES:
//   ✅ 2× more torque (7.09 vs 3.60 N⋅m)
//   ✅ 45% more power (500W vs 347W)
//   ✅ Lighter (395g vs 443g)
//   ✅ Faster spin-up
//   ✅ Better speed maintenance
//
// ================================================================================