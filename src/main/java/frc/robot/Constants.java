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
    public static final double feedTime = .2;
    public static final double subwooferTopRPM = 2400;
    public static final double subwooferBottomRPM = 2400;
  }

  public static class CAN_IDS {
    public static final int FRONT_LEFT_DRIVE = 6;
    public static final int BACK_LEFT_DRIVE = 7;
    public static final int FRONT_RIGHT_DRIVE = 2;
    public static final int BACK_RIGHT_DRIVE = 3;
    public static final int CONVEYOR = 10;
    public static final int INTAKE = 1;
    public static final int SHOOTER_TOP = 9;
    public static final int SHOOTER_BOTTOM = 8;
    public static final int ARM = 4;
    public static final int CLIMBER = 5;

    public static final int COLOR_ID = 0;
    public static final int DISTANCE_ID = 0;

  }

  //public static final int kDriverControllerPort = 0;

  public class Field {
    public final static double speakerTagHeight = 56;
    public final static double sourceTagHeight = 20;
  }

  public static class Controller {
    public static final double deadband = 0.05;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class Robot {
    public static double SteadySpeedRPM = 3000;
    public static final double RPMOffset = 500;
    public static final double slopOffset = 3.4;
    public static final int shooterTolerance = 30;
    public static final double autonIntakeEffort = -0.7;
    public static final double intakeEffort = -0.6;
    public static final double conveyorEffort = .5;
    public static final double ksVolts = 0;//0.18597; // //// //0.29328;  
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kvVoltSecondsPerMeter =2.5656; //2.2846;// //2.17882;
    public static final double kTrackwidthMeters = 0.57; //.67;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);
    public static final double subWooferAngle = 35; //Check
    //public static final double shoulderGearRatio = 10.71;
    public static final double limeLightOffSet = 8.5;
    public static final double kPDriveVel = 0.1884; //0.98;
    public static final double kWheelDiameter = Units.inchesToMeters(6);
    public static final double kGearRatio = 10.86;
    public static final double kLinearDistanceConversionFactor = (Math.PI * kWheelDiameter / kGearRatio);
    public final static double limelightHeight = 21.2;
    public final static double limelightAngle = 26;
    public final static double initialShoulderAngle = -28.5; //Check
    public final static double shoulderGearRatio = 1.0/100.0; //Check
    public final static double ampAngle = 95; //Check
    public final static double climberMaxEncoderPosition = -288;
    //public final static double shooterP = .1;
    //public static final double shooterMargins = 2;
  }
}
