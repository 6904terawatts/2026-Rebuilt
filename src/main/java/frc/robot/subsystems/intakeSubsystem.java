// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakingConstants. *;



public class intakeSubsystem extends SubsystemBase {

  private final SparkFlex intakeTopRoller;
  private final SparkFlex intakebottomRoller;
  /** Creates a new ExampleSubsystem. */
  public intakeSubsystem() {

    intakeTopRoller = new SparkFlex(INTAKE_TOP_ROLLER_ID, MotorType.kBrushed);
    intakebottomRoller = new SparkFlex(INTAKE_BOTTOM_ROLLER_ID, MotorType.kBrushed);



    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeTopRoller.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakebottomRoller.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



  }


  public void setIntakein(double speed){
    intakeTopRoller.set(speed);
    intakebottomRoller.set(speed);
  }

  public void stop(){
  intakeTopRoller.set(0);
  intakebottomRoller.set(0);

  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
