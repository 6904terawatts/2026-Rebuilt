
package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.controls.PoseControls;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  // ============================================================================
  // SUBSYSTEM INITIALIZATION
  // ============================================================================
  
  // Core subsystems
  public final SwerveSubsystem drivebase = new SwerveSubsystem();
  
  // Vision subsystem for AprilTag tracking and pose estimation
  // This MUST be created before being passed to other subsystems
  private final VisionSubsystem vision = new VisionSubsystem();
  
  // Shooting mechanism subsystems
  private final TurretSubsystem turret = new TurretSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final HopperSubsystem hopper = new HopperSubsystem();
  private final KickerSubsystem kicker = new KickerSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();

  // Superstructure coordinates all shooting mechanisms
  private final Superstructure superstructure = new Superstructure(shooter, turret, hood, intake, hopper, kicker);

  private final SendableChooser<Command> autoChooser;

  // Track current alliance for change detection
  private Alliance currentAlliance = Alliance.Red;

  /**
   * The container for the robot. Contains subsystems, I/O devices, and commands.
   */
  public RobotContainer() {
    // ========================================================================
    // VISION INTEGRATION - Wire everything together!
    // ========================================================================
    
    // Connect vision to swerve for pose fusion
    // This allows the drivetrain to use vision estimates with wheel odometry
    drivebase.setVisionSubsystem(vision);
    
    // Connect vision and swerve to superstructure for vision-based aiming
    // This allows the shooter to automatically aim using vision
    superstructure.setVisionSubsystem(vision);
    superstructure.setSwerveSubsystem(drivebase);
    
    // ========================================================================
    // STANDARD INITIALIZATION
    // ========================================================================
    
    // Configure the trigger bindings
    configureBindings();
    buildNamedAutoCommands();

    // Initialize alliance (default to red if not present)
    onAllianceChanged(getAlliance());

    // Set up trigger to detect alliance changes
    new Trigger(() -> getAlliance() != currentAlliance)
        .onTrue(Commands.runOnce(() -> onAllianceChanged(getAlliance())).ignoringDisable(true));

    // Triggers for auto aim/pass poses
    new Trigger(() -> isInAllianceZone())
        .onChange(Commands.runOnce(() -> onZoneChanged()).ignoringDisable(true));

    new Trigger(() -> isOnAllianceOutpostSide())
        .onChange(Commands.runOnce(() -> onZoneChanged()).ignoringDisable(true));

    if (!Robot.isReal() || true) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then
    // stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(10));

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Set up controllers
    DriverControls.configure(ControllerConstants.kDriverControllerPort, drivebase, superstructure);
    OperatorControls.configure(ControllerConstants.kOperatorControllerPort, drivebase, superstructure);
    PoseControls.configure(ControllerConstants.kPoseControllerPort, drivebase);


  





  }

  private void buildNamedAutoCommands() {
    // Add any auto commands to the NamedCommands here
    

    NamedCommands.registerCommand("driveBackwards",
        drivebase.driveBackwards().withTimeout(1)
            .withName("Auto.driveBackwards"));

    NamedCommands.registerCommand("driveForwards",
        drivebase.driveForward().withTimeout(2)
            .withName("Auto.driveForwards"));


    NamedCommands.registerCommand("Shoot", 
   shooter.spinUp().withTimeout(5)
   .withName("Auto.shoot"));      
   
   NamedCommands.registerCommand("Intake",
    intake.intakeCommand().withTimeout(5)
    .withName("auto.Intake"));

    // NamedCommands.registerCommand("Climb" ,
    // climber.climb().withTimeout(1)
    // .withName("Auto.CLimb"));


  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }

  /**
   * Compatibility method for simulation - returns the swerve drive subsystem
   * This is needed because the simulation code in Robot.java calls getSwerveDrive()
   * to access the MapleSim drive for physics simulation.
   */
  public SwerveSubsystem getSwerveDrive() {
    return drivebase;
  }

  public Pose2d getRobotPose() {
    return drivebase.getPose();
  }

  public Pose3d getAimDirection() {
    // Apply robot heading first, then turret/hood rotation on top
    Pose3d shooterPose = superstructure.getShooterPose();

    var pose = drivebase.getPose3d().plus(new Transform3d(
        shooterPose.getTranslation(), shooterPose.getRotation()));

    return pose;
  }

  public Translation3d getAimPoint() {
    return superstructure.getAimPoint();
  }

  public void setAimPoint(Translation3d aimPoint) {
    superstructure.setAimPoint(aimPoint);
  }

  private Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Red);
  }

  private boolean isInAllianceZone() {
    Alliance alliance = getAlliance();
    Distance blueZone = Inches.of(182);
    Distance redZone = Inches.of(469);

    if (alliance == Alliance.Blue && drivebase.getPose().getMeasureX().lt(blueZone)) {
      return true;
    } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureX().gt(redZone)) {
      return true;
    }

    return false;
  }

  private boolean isOnAllianceOutpostSide() {
    Alliance alliance = getAlliance();
    Distance midLine = Inches.of(158.84375);

    if (alliance == Alliance.Blue && drivebase.getPose().getMeasureY().lt(midLine)) {
      return true;
    } else if (alliance == Alliance.Red && drivebase.getPose().getMeasureY().gt(midLine)) {
      return true;
    }

    return false;
  }

  private void onZoneChanged() {
    if (isInAllianceZone()) {
      superstructure.setAimPoint(Constants.AimPoints.getAllianceHubPosition());
    } else {
      if (isOnAllianceOutpostSide()) {
        superstructure.setAimPoint(Constants.AimPoints.getAllianceOutpostPosition());
      } else {
        superstructure.setAimPoint(Constants.AimPoints.getAllianceFarSidePosition());
      }
    }
  }

  private void onAllianceChanged(Alliance alliance) {
    currentAlliance = alliance;

    // Update aim point based on alliance
    if (alliance == Alliance.Blue) {
      superstructure.setAimPoint(Constants.AimPoints.BLUE_HUB.value);
    } else {
      superstructure.setAimPoint(Constants.AimPoints.RED_HUB.value);
    }

    System.out.println("Alliance changed to: " + alliance);
  }
}
