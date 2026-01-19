// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  public void arcadeDrive(double leftSpeed, double rightSpeed) {
    leftLeader.setControl(leftOut.withOutput(leftSpeed));
    rightLeader.setControl(rightOut.withOutput(rightSpeed));
  }

  public Command ArcadeDriveCommand(double leftSpeed, double rightSpeed) {
    return run(
        () -> {
          arcadeDrive(leftSpeed, rightSpeed);
        });
  }
}
