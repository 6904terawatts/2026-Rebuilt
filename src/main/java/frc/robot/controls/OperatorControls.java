package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class OperatorControls {
  public static final boolean MACOS_WEIRD_CONTROLLER = true;

  public static void configure(int port, SwerveSubsystem drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);
    // CommandGenericHID buttonBoard = new CommandGenericHID(port);
    // if (Robot.isSimulation()) {
    // controller.leftBumper().whileTrue(aimCommand(drivetrain, superstructure));
    // controller.start().whileTrue(fireAlgae(drivetrain, superstructure));

    // Commands.run(() -> {
    // double leftX = controller.getLeftX();
    // double leftY = controller.getLeftY();
    // double rightY = controller.getRightY();

    // if (MACOS_WEIRD_CONTROLLER) {
    // rightY = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    // }

    // // Apply deadband
    // if (Math.abs(leftX) < Constants.ControllerConstants.DEADBAND)
    // leftX = 0;
    // if (Math.abs(leftY) < Constants.ControllerConstants.DEADBAND)
    // leftY = 0;
    // if (Math.abs(rightY) < Constants.ControllerConstants.DEADBAND)
    // rightY = 0;

    // Translation3d translation = new Translation3d(leftX, leftY, rightY);

    // if (MACOS_WEIRD_CONTROLLER) {
    // // MacOS Xbox controller mapping is weird - swap X and Y
    // translation = new Translation3d(leftY, leftX, rightY);
    // }

    // // System.out.println("Adjusting pose by: " + translation.toString());

    // var newAimPoint = superstructure.getAimPoint().plus(translation.times(0.05));
    // // new Transform3d(leftX * 0.05, leftY * 0.05, rightY * 0.05));

    // superstructure.setAimPoint(newAimPoint);
    // }).ignoringDisable(true).schedule();
    // }

    // REAL CONTROLS
    // controller.start().onTrue(superstructure.rezeroIntakePivotAndTurretCommand().ignoringDisable(true));

    // buttonBoard.button(9).onTrue(superstructure.rezeroIntakePivotAndTurretCommand().ignoringDisable(true));

    // controller.rightBumper() //button 6
    // .whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));

    controller.b().whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));

    // controller.leftBumper().toggleOnTrue( // button 5
    // new ShootOnTheMoveCommand(drivetrain, superstructure, () ->
    // superstructure.getAimPoint())
    // .ignoringDisable(true)
    // .withName("OperatorControls.aimCommand"));

    // buttonBoard.button(5).toggleOnTrue( // button 5
    // new ShootOnTheMoveCommand(drivetrain, superstructure, () ->
    // superstructure.getAimPoint())
    // .ignoringDisable(true)
    // .withName("OperatorControls.aimCommand"));

    // controller.y().toggleOnTrue(superstructure.shootCommand());// button 4
    // controller.x().whileTrue(superstructure.stopShootingCommand()); //button3

    controller.a().whileTrue(superstructure.shootCommand());
    controller.y().whileTrue(superstructure.stopShootingCommand());

    // controller.a().whileTrue( //button 1
    // superstructure.feedAllCommand()
    // .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

    controller.rightBumper().whileTrue( // button 1
        superstructure.feedAllCommand()
            .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

    // controller.b().whileTrue( //button 2
    // superstructure.backFeedAllCommand()
    // .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

    controller.leftBumper().whileTrue( // button 2
        superstructure.backFeedAllCommand()
            .finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

    controller.rightTrigger().whileTrue(superstructure.feedAllCommand().andThen(Commands.waitSeconds(3))
        .andThen(superstructure.shootCommand()).finallyDo(() -> superstructure.stopFeedingAllCommand().schedule()));

    // buttonBoard.button(12).onTrue(superstructure.setTurretRight().withName("ButtonBoard.setTurretRight"));

    // controller.rightTrigger().whileTrue(superstructure.intakeCommand());
    // //button7
    controller.x().whileTrue(superstructure.intakeCommand()); // button7

    // controller.leftTrigger().whileTrue(superstructure.ejectCommand()); // button
    // 8
    controller.leftTrigger().whileTrue(superstructure.ejectCommand());

    // buttonBoard.button(9).toggleOnTrue(superstructure.passCommand());

  }
}
