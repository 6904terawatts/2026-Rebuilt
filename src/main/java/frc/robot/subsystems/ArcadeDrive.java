// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArcadeDrive extends SubsystemBase {
  /** Creates a new TankDrive. */
  private final CANBus kCANBus = new CANBus("rio");

  private final TalonFX leftLeader = new TalonFX(22, kCANBus);
  private final TalonFX leftFollower = new TalonFX(24, kCANBus);
  private final TalonFX rightLeader = new TalonFX(21, kCANBus);
  private final TalonFX rightFollower = new TalonFX(20, kCANBus);

  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  public ArcadeDrive() {

    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();

    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftLeader.getConfigurator().apply(leftConfiguration);
    leftFollower.getConfigurator().apply(leftConfiguration);
    rightLeader.getConfigurator().apply(rightConfiguration);
    rightFollower.getConfigurator().apply(rightConfiguration);

    /* Set up followers to follow leaders */
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    leftLeader.setSafetyEnabled(true);
    rightLeader.setSafetyEnabled(true);
  }

  @Override
   public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about its center) and uses these to control the drivetrain motors
   */
  public void arcadeDrive(double speed, double rotation) {
    // Calculate left and right outputs from speed and rotation
    double leftOutput = speed + rotation;
    double rightOutput = speed - rotation;
    
    // Apply deadband
    if (Math.abs(leftOutput) < Constants.kArcadeDeadBand) {
      leftOutput = 0;
    }
    if (Math.abs(rightOutput) < Constants.kArcadeDeadBand) {
      rightOutput = 0;
    }
    
    // Square the outputs while preserving sign (for finer control)
    double squaredLeftOutput = leftOutput * leftOutput * Math.signum(leftOutput);
    double squaredRightOutput = rightOutput * rightOutput * Math.signum(rightOutput);
    
    // Send to motors
    leftLeader.setControl(leftOut.withOutput(squaredLeftOutput));
    rightLeader.setControl(rightOut.withOutput(squaredRightOutput));
  }


 public Command ArcadeDriveCommand(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
  return run(() -> {
    arcadeDrive(leftSpeed.getAsDouble(), rightSpeed.getAsDouble());
  });
}
}
