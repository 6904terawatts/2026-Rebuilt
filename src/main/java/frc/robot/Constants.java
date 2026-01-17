// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;





  }
  public static final class IntakingConstants{
public static final int INTAKE_BOTTOM_ROLLER_ID = 4;
public static final int INTAKE_TOP_ROLLER_ID = 5;
public static final double INTAKING_INTAKE_SPEED = 0.2;

}

public static final class ShootingConstants{


    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information

    public static final double LAUNCHING_FEEDER_SPEED = .4;
    public static final double LAUNCHING_LAUNCHER_SPEED = .6;
    public static final double SPIN_UP_FEEDER_SPEED = -0.4;
    public static final double SPIN_UP_SECONDS = 2;
    public static final int LAUNCH_WHEEL_ID = 6;
    public static final int FEEDER_WHEEL_ID = 7;


}
}
