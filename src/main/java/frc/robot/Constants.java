// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;

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

  /**
   * Input/Output constants
   */
  public static final class IOConstants {
    public static final int kDriverControllerPort = 0;

    public static final double kControllerDeadband = 0.15;
    public static final double kSlowModeScalar = 0.8;
  }

  public static final class DriveConstants {
    // TODO: set motor and encoder constants
    public static final int kFrontLeftDriveMotorPort = 4;
    public static final int kRearLeftDriveMotorPort = 5;
    public static final int kFrontRightDriveMotorPort = 12;
    public static final int kRearRightDriveMotorPort = 34;

    public static final int kFrontLeftTurningMotorPort = 11;
    public static final int kRearLeftTurningMotorPort = 9;
    public static final int kFrontRightTurningMotorPort = 36;
    public static final int kRearRightTurningMotorPort = 16;

    public static final int kFrontLeftTurningEncoderPort = 19;
    public static final int kRearLeftTurningEncoderPort = 20;
    public static final int kFrontRightTurningEncoderPort = 18;
    public static final int kRearRightTurningEncoderPort = 17;

    // TODO: Test motor orientations before driving on an actual robot
    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    /** Distance between centers of right and left wheels on robot (in meters). */
    public static final double kTrackWidth = 0.5;

    /** Distance between front and back wheels on robot (in meters). */
    public static final double kWheelBase = 0.5;

    /** Diameter of each wheel in the SDS MK4i swerve module (in meters) */
    public static final double kWheelDiameterMeters = 0.1;

    /** Gear ratio between the motor and the wheel. */
    public static final double kDrivingGearRatio = 8.14; // SDS MK4i's in L1 configuration

    // TODO: Tune this PID before running on a robot on the ground
    public static final double kPModuleTurningController = 0.3;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    /** For a a SDS Mk4i L1 swerve base with Neos */
    public static final double kMaxSpeedMetersPerSecond = 3.6576;
    /** For a a SDS Mk4i L1 swerve base with Neos */
    public static final double kMaxAngularSpeedRadiansPerSecond = 15.24 / 3;

    /** Heading Correction */
    public static final double kHeadingCorrectionTurningStopTime = 0.2;
    // TODO: Tune this PID before running on a robot on the ground
    public static final double kPHeadingCorrectionController = 5;
  }

  public static final class VisionConstants {
    public static final double kF = 0.3048;
    public static final double kS = 0.254;
    public static final double kU = 0;
    public static final double kR = 0;
    public static final double kP = 0;
    public static final double kY = 0;

    public static final double kMaxAmbiguityNonMega = 0.7;
    public static final double kMaxDistNonMega = 3;

    public static final double kdtThreshold = 40; // max delta timestamp for past measurement to be valid. set to zero
                                                  // to disable angle rate consideration

    // 1foot forward of center, 10 inch right of center, no height
    public static final String kLimelightName = "limelight";

    public static final Vector<N3> kOdometrySTDDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Vector<N3> kVisionSTDDevs = VecBuilder.fill(0.7, 0.7, Integer.MAX_VALUE);

    public static final int kIMUType_external = 0;
    public static final int kIMUType_sync = 1;
    public static final int kIMUType_internal = 2;

    public static final class VirtualLimelightConstants {
      public static final double[] kHWMetrics = new double[] {-1, -1, -1, -1}; //used for detecting real limelight

      public static final double kLatency = 0.01;

      public static final int kbotpose_orb_wpiblue_header_size = 11;
      public static final int kValsPerFiducial = 7;
    }
  }

}
