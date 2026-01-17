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
import static frc.robot.Constants.ShootingConstants. *;
public class shooterSubsystem extends SubsystemBase {

  private final SparkFlex feederWheel;
  private final SparkFlex launchWheel;

  /** Creates a new ExampleSubsystem. */
  public shooterSubsystem() {

  feederWheel = new SparkFlex(FEEDER_WHEEL_ID, MotorType.kBrushed);
  launchWheel = new SparkFlex(LAUNCH_WHEEL_ID, MotorType.kBrushed);

     SparkFlexConfig shooterConfig = new SparkFlexConfig();
    feederWheel.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launchWheel.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);




    
  }



   public void setLauncherRoller(double speed) {
    launchWheel.set(speed);
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double speed) {
    feederWheel.set(speed);
  }

  // A method to stop the rollers
  public void stop() {
    feederWheel.set(0);
    launchWheel.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
