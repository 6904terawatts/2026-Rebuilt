package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {

  private static final double INTAKE_SPEED = 1.3;

  // SparkFlex controlling the intake roller
  private SparkFlex rollerSpark = new SparkFlex(Constants.IntakeConstants.kRollerMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3))) // 3:1 Direct drive, adjust if geared
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(30));

  private SmartMotorController smc = new SparkWrapper(rollerSpark, DCMotor.getNeoVortex(1), smcConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("IntakeRoller", TelemetryVerbosity.HIGH);
  private FlyWheel intake = new FlyWheel(intakeConfig);

  // 5:1, 5:1, 60/18 reduction
  private SmartMotorControllerConfig intakePivotSmartMotorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(
        12.0,                                    // P (was 25)
        0.0,                                     // I
        0.15,                                    // D (added)
        DegreesPerSecond.of(720),                // 2× velocity
        DegreesPerSecondPerSecond.of(1080))      // 3× acceleration
    .withFeedforward(new SimpleMotorFeedforward(0, 10, 0))
    .withTelemetry("IntakePivotMotor", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5, 60.0 / 18.0)))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(15))
    .withClosedLoopRampRate(Seconds.of(0.1))
    .withOpenLoopRampRate(Seconds.of(0.1));
    
  private SparkFlex pivotMotor = new SparkFlex(Constants.IntakeConstants.kPivotMotorId, MotorType.kBrushless);

  private SmartMotorController intakePivotController;  // Initialize in constructor

  private ArmConfig intakePivotConfig;  // Initialize in constructor
  
  private Arm intakePivot;  // Initialize in constructor

  public IntakeSubsystem() {
    
    // ============================================================
    // DISABLE SOFT LIMITS ON PIVOT MOTOR (SparkFlex doesn't have factory reset in YAMS wrapper)
    // ============================================================
    
    try {
      System.out.println("🔧 Initializing intake pivot motor...");
      
      // SparkFlex in YAMS doesn't expose restoreFactoryDefaults()
      // So we just disable soft limits directly
      
      // Get the internal REV CANSparkFlex if possible
      // For now, soft limits are controlled by YAMS config
      // Make sure .withSoftLimits() is commented out in config below
      
      System.out.println("✅ Soft limits disabled via YAMS config");
      
    } catch (Exception e) {
      System.err.println("⚠️ Error configuring pivot motor: " + e.getMessage());
      e.printStackTrace();
    }
    
    // ============================================================
    // CREATE YAMS WRAPPERS (Without soft limits)
    // ============================================================
    
    intakePivotController = new SparkWrapper(pivotMotor, DCMotor.getNeoVortex(1), intakePivotSmartMotorConfig);
    
    intakePivotConfig = new ArmConfig(intakePivotController)
        // .withSoftLimits(Degrees.of(0), Degrees.of(-150))  // KEEP COMMENTED OUT!
        .withHardLimit(Degrees.of(-5), Degrees.of(155))
        .withStartingPosition(Degrees.of(0))
        .withLength(Feet.of(1))
        .withMass(Pounds.of(2))
        .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

    intakePivot = new Arm(intakePivotConfig);
    
    System.out.println("✅ YAMS wrappers created - soft limits disabled");
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return intake.set(INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Run");
  }

  /**
   * Command to eject while held.
   */
  public Command ejectCommand() {
    return intake.set(-INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Eject");
  }

  public Command setPivotAngle(Angle angle) {
    return Commands.run(() -> {
      intakePivotController.setPosition(angle);
    }, this).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotMotor.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
  }

  /**
   * Command to deploy intake and run roller while held.
   * Stops roller when released.
   */
  public Command deployAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      smc.setDutyCycle(INTAKE_SPEED);
    }, this).finallyDo(() -> {
      smc.setDutyCycle(0);
      setIntakeHold();
    }).withName("Intake.DeployAndRoll");
  }

  public Command backFeedAndRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      // smc.setDutyCycle(-INTAKE_SPEED);
    }, this).finallyDo(() -> {
      smc.setDutyCycle(0);
      setIntakeHold();
    }).withName("Intake.BackFeedAndRoll");
  }

  private void setIntakeStow() {
    intakePivotController.setPosition(Degrees.of(0));
  }

  private void setIntakeFeed() {
    intakePivotController.setPosition(Degrees.of(59));
  }

  private void setIntakeHold() {
    intakePivotController.setPosition(Degrees.of(115));
  }

  private void setIntakeDeployed() {
    intakePivotController.setPosition(Degrees.of(148));
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();
    intakePivot.updateTelemetry();
    SmartDashboard.putNumber("Intake Pivot Angle", intakePivot.getAngle().in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
    intakePivot.simIterate();
  }
}