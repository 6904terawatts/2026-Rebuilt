// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.IntakingConstants.*;

/** An example command that uses an example subsystem. */
public class intake extends Command {
  intakeSubsystem intakeSystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public intake(intakeSubsystem intakeSystem) {
    addRequirements(intakeSystem);
    this.intakeSystem = intakeSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSystem.setIntakein(SmartDashboard.getNumber("Intaking in speed", INTAKING_INTAKE_SPEED));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSystem.setIntakein(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
