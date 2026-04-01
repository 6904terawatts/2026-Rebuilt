package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;

/**
 * Simple Auto: Backup and Shoot
 * 
 * 1. Spin up shooter (4000 RPM)
 * 2. Back up for 2 seconds
 * 3. Wait 2 seconds
 * 4. Feed all balls for 5 seconds
 * 5. Stop everything
 */
public class BackupAndShootAuto {
  
  /**
   * Create the backup and shoot auto command
   */
  public static Command create(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      HopperSubsystem hopper,
      KickerSubsystem kicker) {
    
    return Commands.sequence(
        // ================================================================
        // STEP 1: Spin up shooter to 4000 RPM
        // ================================================================
        Commands.runOnce(() -> {
          System.out.println("🚀 AUTO: Starting shooter spin-up...");
        }),
        
        Commands.parallel(
            shooter.spinUp(),  // Start spinning shooter
            
            Commands.sequence(
                // Wait a moment for shooter to start
                Commands.waitSeconds(0.5),
                
                // ================================================================
                // STEP 2: Back up for 2 seconds
                // ================================================================
                Commands.runOnce(() -> {
                  System.out.println("⬅️ AUTO: Backing up...");
                }),
                
                swerve.driveBackwards().withTimeout(0),
                
                Commands.runOnce(() -> {
                  System.out.println("🛑 AUTO: Stopped backing up");
                }),
                
                // ================================================================
                // STEP 3: Wait 2 seconds (shooter still spinning)
                // ================================================================
                Commands.runOnce(() -> {
                  System.out.println("⏳ AUTO: Waiting 2 seconds...");
                }),
                
                Commands.waitSeconds(2.0),
                
                // ================================================================
                // STEP 4: Feed all balls for 5 seconds
                // ================================================================
                Commands.runOnce(() -> {
                  System.out.println("🎯 AUTO: Feeding balls!");
                }),
                
                Commands.parallel(
                    hopper.feedCommand(),
                    kicker.feedCommand()
                ).withTimeout(5.0),
                
                Commands.runOnce(() -> {
                  System.out.println("✅ AUTO: Finished feeding");
                })
            )
        ),
        
        // ================================================================
        // STEP 5: Stop everything
        // ================================================================
        Commands.runOnce(() -> {
          System.out.println("🛑 AUTO: Stopping all mechanisms");
        }),
        
        Commands.parallel(
            shooter.stop(),
            hopper.stopCommand(),
            kicker.stopCommand()
        ),
        
        Commands.runOnce(() -> {
          System.out.println("✅ AUTO: Complete!");
        })
        
    ).withName("BackupAndShoot");
  }
}