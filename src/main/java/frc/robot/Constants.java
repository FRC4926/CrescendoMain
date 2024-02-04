// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // public static final double kTurnRateConstant = .7;
  public static class Auton {
    public static final double startingX = 0;
    public static final double startingY = 2;
  }

  public static class CAN_IDS {
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int BACK_LEFT_DRIVE = 2;
    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final int BACK_RIGHT_DRIVE = 4;
    public static final int CONVEYOR = 6;
    public static final int INTAKE = 8;
    public static final int SHOOTER_TOP = 7;
    public static final int SHOOTER_BOTTOM = 5;
    public static final int ARM_LEFT = 9;
    public static final int ARM_RIGHT = 10;

    public static final int COLOR_ID = 7;

  }

  public static final int kDriverControllerPort = 0;

  public class Field {
    public final static double speakerTagHeight = 51.875;
  }

  public static class Controller {
    public static final double deadband = 0.02;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class Robot {
    public static final double ksVolts = 0.17882;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kvVoltSecondsPerMeter = 2.6868;
    public static final double kTrackwidthMeters = .67;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);
    public static final double kPDriveVel = .98;
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kGearRatio = 10.86;
    public static final double kLinearDistanceConversionFactor = (Math.PI * kWheelDiameter / kGearRatio);
    public final static double cameraHeight = 11.5;
    public final static double cameraAngle = 0;
    public final static double initialShoulderAngle = 909;
    public final static double wristAngle = 909;
    public final static double shoulderGearRation = 1 / 156;
    public final static double ampAngle = 909;
  }

}
