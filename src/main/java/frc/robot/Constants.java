// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
 // public static final double kTurnRateConstant = .7;
 public static class autonConstants{
    public static final double startingX=0;
    public static final double startingY=2;
 }
  public static class CAN_IDS {
    public static final int FRONT_LEFT = 1;
    public static final int BACK_LEFT = 2;
    public static final int FRONT_RIGHT = 3;
    public static final int BACK_RIGHT = 4;
    
  }
 public static final int kDriverControllerPort = 0;


public class FieldConstant{
  public final static double speakerTagHeight = 51.875;
}

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class RobotParameters {
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kGearRatio = 10.86;
    public static final double kLinearDistanceConversionFactor = (Math.PI*kWheelDiameter/kGearRatio);
    public final static double cameraHeight = 11.5;
    public final static double cameraAngle = 0;

}
    
  }
