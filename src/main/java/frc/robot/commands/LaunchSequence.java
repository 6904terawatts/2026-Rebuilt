// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooterSubsystem;
import static frc.robot.Constants.ShootingConstants. *;

public class LaunchSequence extends SequentialCommandGroup {
  /** Creates a new LaunchSequence. */
  public LaunchSequence(shooterSubsystem shootSystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SpinUp(shootSystem).withTimeout(SPIN_UP_SECONDS),
        new Launch(shootSystem));
  }
}
