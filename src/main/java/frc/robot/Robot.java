package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CommandsLogging;
import frc.robot.util.maplesim.Arena2026Rebuilt;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private SimulatedArena arena;

  public Robot() {
    Logger.recordMetadata("ProjectName", "CA_Ri3D_2026"); // Set a metadata value

    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    if (isReal()) {
      // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    }

    Logger.start();

    // Register command logging callbacks with CommandScheduler
    CommandScheduler.getInstance().onCommandInitialize(CommandsLogging::commandStarted);
    CommandScheduler.getInstance().onCommandFinish(CommandsLogging::commandEnded);
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (interrupted, interrupting) -> {
              interrupting.ifPresent(
                  interrupter -> CommandsLogging.runningInterrupters.put(interrupter, interrupted));
              CommandsLogging.commandEnded(interrupted);
            });

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Log running commands and subsystem requirements
    CommandsLogging.logRunningCommands();
    CommandsLogging.logRequiredSubsystems();

    if (Robot.isSimulation()) {
      Pose3d[] fuelPoses = arena.getGamePiecesArrayByType("Fuel");
      Logger.recordOutput("FieldSimulation/FuelPoses", fuelPoses);
    }

    Logger.recordOutput("FieldSimulation/RobotPose", m_robotContainer.getRobotPose());
    Logger.recordOutput("FieldSimulation/TargetPose",
        m_robotContainer.getDrivebase().field.getObject("targetPose").getPose());
    Logger.recordOutput("FieldSimulation/AimDirection", m_robotContainer.getAimDirection());
    Logger.recordOutput("FieldSimulation/AimTarget", new Pose3d(m_robotContainer.getAimPoint(), Rotation3d.kZero));
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    // Shut down the old arena instance first to release ownership of all bodies
    // (including the drivetrain) so they can be added to a new physics world
    SimulatedArena.getInstance().shutDown();

    SimulatedArena.overrideInstance(new Arena2026Rebuilt());

    arena = SimulatedArena.getInstance();

    // Use reflection to safely attempt to retrieve a MapleSim drive instance from the drivebase
    // This avoids a compile-time dependency on a method that may not exist on SwerveSubsystem.
    try {
      Object drivebase = m_robotContainer.getDrivebase();

      java.lang.reflect.Method getter = null;
      try {
        getter = drivebase.getClass().getMethod("getMapleSimDrive");
      } catch (NoSuchMethodException e) {
        // Method not present; nothing to add
      }

      if (getter != null) {
        Object mapleHolder = getter.invoke(drivebase);
        Object mapleInstance = null;

        // Unwrap common holder types
        if (mapleHolder instanceof java.util.Optional) {
          java.util.Optional<?> opt = (java.util.Optional<?>) mapleHolder;
          if (opt.isPresent()) {
            mapleInstance = opt.get();
          }
        } else if (mapleHolder instanceof java.util.function.Supplier) {
          mapleInstance = ((java.util.function.Supplier<?>) mapleHolder).get();
        } else {
          mapleInstance = mapleHolder;
        }

        if (mapleInstance != null) {
          // Find an addDriveTrainSimulation method that takes one parameter and invoke it via reflection
          java.lang.reflect.Method addMethod = null;
          for (java.lang.reflect.Method m : arena.getClass().getMethods()) {
            if (m.getName().equals("addDriveTrainSimulation") && m.getParameterCount() == 1) {
              addMethod = m;
              break;
            }
          }
          if (addMethod != null) {
            addMethod.invoke(arena, mapleInstance);
          } else {
            System.err.println("addDriveTrainSimulation method not found on SimulatedArena");
          }
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void simulationPeriodic() {
    arena.simulationPeriodic();
  }
}
