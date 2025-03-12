// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

  public static final double kFastPeriodicPeriod = 0.01; // 100Hz, 10ms
  public static final double kFastPeriodicOfset = 1e-6; // 1 micro second, no noticable effect other than scheduling
                                                        // order

  /**
   * Input/Output constants
   */
  public static final class IOConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kControllerDeadband = 0.15;
    public static final double kSlowModeScalar = 0.8;

    public static final double kElevatorAxisScalar = 0.15; //TODO: tune
    public static final double kPivotAxisScalar = -0.25; //TODO: tune

    public static final int kDPadUp = 0;
    public static final int kDPadRight = 90;
    public static final int kDPadDown = 180;
    public static final int kDPadLeft = 270;
  }

  public static final class DriveConstants {
    // TODO: set motor and encoder constants
    public static final int kFrontLeftDriveMotorPort = 32;
    public static final int kRearLeftDriveMotorPort = 29;
    public static final int kFrontRightDriveMotorPort = 36;
    public static final int kRearRightDriveMotorPort = 34;

    public static final int kFrontLeftTurningMotorPort = 28;
    public static final int kRearLeftTurningMotorPort = 22;
    public static final int kFrontRightTurningMotorPort = 37;
    public static final int kRearRightTurningMotorPort = 26;

    public static final int kFrontLeftTurningEncoderPort = 5;
    public static final int kRearLeftTurningEncoderPort = 6;
    public static final int kFrontRightTurningEncoderPort = 3;
    public static final int kRearRightTurningEncoderPort = 4;

    // TODO: Test motor orientations before driving on an actual robot
    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    /** Distance between centers of right and left wheels on robot (in meters). */
    public static final double kTrackWidth = 0.57785;

    /** Distance between front and back wheels on robot (in meters). */
    public static final double kWheelBase = 0.57785;

    /** Diameter of each wheel in the SDS MK4i swerve module (in meters) */
    public static final double kWheelDiameterMeters = 0.1;

    /** Gear ratio between the motor and the wheel. */
    public static final double kDrivingGearRatio = 6.12; // SDS MK4i's in L2 configuration

    // TODO: Tune this PID before running on a robot on the ground
    public static final double kPModuleTurningController = 0.3;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    /** For a a SDS Mk4i L1 swerve base with Neos */
    public static final double kMaxSpeedMetersPerSecond = 4.4196;

    // TODO: Set max acceleration constants
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    
    /** For a a SDS Mk4i L1 swerve base with Neos */
    public static final double kMaxAngularSpeedRadiansPerSecond = 10.8164;

    /** Heading Correction */
    public static final double kHeadingCorrectionTurningStopTime = 0.2;
    // TODO: Tune this PID before running on a robot on the ground
    public static final double kPHeadingCorrectionController = 5;

    public static final boolean kAutoDriving = true;
    
    // TODO: set these on real robot
    public static final double kMaxAccelerationUnitsPerSecond = 100;
    public static final double kMaxAngularAccelerationUnitsPerSecond = 100;
  }

  public static final class VisionConstants {
    // TODO: Update cam pose relative to center of bot
    public static final Pose3d kCamPos = new Pose3d(
      // new Translation3d(0.3048,0.254,0),
      new Translation3d(0.34925, -0.2413, 0.2),
      new Rotation3d(180,10,0)
    );

    public static final String kLimelightName = "limelight";

    // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
    public static final int kIMUMode = 0;

    // TODO: Experiment with different std devs in the pose estimator
    public static final Vector<N3> kOdometrySTDDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Vector<N3> kVisionSTDDevs = VecBuilder.fill(0.7, 0.7, 999999);

    public static final boolean kUseVision = true;
  }

  public static final class ElevatorConstants {
    // TODO: Set motor and distance sensor ports
    public static final int kElevatorMotorPort = 50;
    public static final int kElevatorCANrangePort = 9;

    // TODO: Tune PID for elevator
    public static final double kPElevator = 0.9;
    public static final double kMaxV = 30;
    public static final double kMaxA = 30;

    // TODO: Set these constants
    public static final double kElevatorGearing = 0.2; //20 rot = 4 inch of first stage
    // public static final double kElevatorUpMaxSpeed = 0.6;
    public static final double kElevatorUpMaxSpeed = 1;

    public static final double kElevatorDownMaxSpeed = -0.45;
    public static final double kElevatorFeedForward = 0.03;
    public static final double kElevatorSpeedScalar = 1;
    public static final double kElevatorBottom = 0.2;
    public static final double kElevatorTop = 21;
    public static final double kElevatorSensorMaxTrustDistance = 10;

    public static final double kPositionTolerance = 0.04; //TODO: tune
    public static final double kVelocityTolerance = 1;

    public static final double kLowHeightSlowdownThreshold = 1;
    public static final double kLowHeightSlowdownMaxSpeed = -.1;

    // inches
    public static final double kSensorOffset = -4.40;

    public static final double kBoundaryHintThreshold = 0.5;
    public static final int kSampleCount = 5;
  }

  public static final class EndEffectorConstants{
    // TODO: Set these constants
    public static final int kPivotMotorPort = 52;
    public static final int kEffectorMotorPort = 53;
    public static final int kEndEffectorCANrangePort = 8;

    public static final double kPEndEffector = 0.5;
    public static final double kPivotMaxSpeedRetract = 0.4;
    public static final double kPivotMaxSpeedExtend = -0.4;

    public static final double kL1Pivot = 0.5;
    public static final double kL23Pivot = 0.5;
    public static final double kL4Pivot = 0.5;

    public static final double kAlgaeIntakeSpeed = 0.75;
    public static final double kCoralIntakeSpeed = -0.25;
    public static final double kAlgaeOuttakeSpeed = -0.5;
    public static final double kCoralOuttakeSpeed = -0.25;

    public static final double kPivotTolerance = 0.05; // pivot tolerance in degrees

    public static final double kSensorDistanceThreshold = .1; // meters, TODO: tune

    public static final double kMinAlgaeExtension = 0.3;

    public static final double kPivotFeedForwards = 0.00;

    /**
     * Radians.
     * Used to round values near the wraparound to zero.
     * Lower numbers are more reliable.
     * Pivot should never physically reach this angle
     */
    public static final double kPivotWraparoundPoint = 0.75 * Math.PI * 2;

    // radians
    public static final double kAgressiveComponent = Math.toRadians(.25);

    /**
     * Holds the safe minimum and maximum limits of end effector's pivot based on
     * elevator height
     * Each key is the starting (from zero) elevator height for the limit. Height is inclusive
     * Each value is a Pair with the minimum and maximum pivot angles (inclusive) in radians,
     * respectively
     * 
     * For example:
     * 
     * Map.ofEntries(
     * Map.entry(-100000.0, Pair.of(0.0, Math.PI / 2)),
     * Map.entry(0.0, Pair.of(0.0, Math.PI / 2)),
     * Map.entry(1.0, Pair.of(Math.PI / 2, Math.PI))
     * Map.entry(100000.0, Pair.of(Math.PI / 2, Math.PI))
     * );
     * 
     * means that:
     * pivot angles between elevator heights [-1, 0) must be from 0 to 90 degrees
     *  this acts as a safeguard for negative values, should be less than min
     *  physical height
     * pivot angles between elevator heights [0, 1) must be from 0 to 90 degrees,
     * pivot angles between elevator heights [1, 5) must be from 90 to 180
     * pivot angles between elevator heights [5, infinity) must be from 90 to 180
     * degrees
     *  this acts as a safeguard for very high values, should be greater than max
     *  physical height
     */
    public static final NavigableMap<Double, List<Pair<Double, Double>>> kSafePivotPositions = new TreeMap<>(
        Map.ofEntries(
            Map.entry(-100000.0, Arrays.asList(Pair.of(0.005 * Math.PI * 2, 0.435 * Math.PI * 2))),
            Map.entry(-10000.0,  Arrays.asList(Pair.of(0.005 * Math.PI * 2, 0.435 * Math.PI * 2))),
            Map.entry(0.20,    Arrays.asList(Pair.of(0.005 * Math.PI * 2, 0.435 * Math.PI * 2))),
            Map.entry(1.1,     Arrays.asList(Pair.of(0.036 * Math.PI * 2, 0.500 * Math.PI * 2))),
            Map.entry(4.6,     Arrays.asList(Pair.of(0.090 * Math.PI * 2, 0.500 * Math.PI * 2))),
            Map.entry(8.1,    Arrays.asList(Pair.of(0.165* Math.PI * 2, 0.500 * Math.PI * 2))),
            Map.entry(13.5,    Arrays.asList(Pair.of(0.067* Math.PI * 2, 0.500 * Math.PI * 2))),
            Map.entry(15.2,    Arrays.asList(Pair.of(0.279* Math.PI * 2, 0.500 * Math.PI * 2),
                                               Pair.of(0.050* Math.PI * 2, 0.118 * Math.PI * 2))),
            Map.entry(10000.0, Arrays.asList(Pair.of(0.279* Math.PI * 2, 0.500 * Math.PI * 2),
                                               Pair.of(0.043* Math.PI * 2, 0.118 * Math.PI * 2))),
            Map.entry(100000.0,Arrays.asList(Pair.of(0.279* Math.PI * 2, 0.500 * Math.PI * 2),
                                               Pair.of(0.043* Math.PI * 2, 0.118 * Math.PI * 2)))
        )); 
  }

  public static final class SetpointConstants {
    public static final double kL1CoralHeight = 0.2;
    public static final double kL2CoralHeight = 4.1;
    public static final double kL3CoralHeight = 10.5;

    public static final double kL4CoralSingleHeight = 13.8;
    public static final double kL4CoralDoubleHeight = 18;

    public static final double kL1CoralAngle = 0.05;
    public static final double kL2CoralAngle = 0.717;
    public static final double kL3CoralAngle = 1.035;

    public static final double kL4CoralNoneAngle = 1.3;
    public static final double kL4CoralSingleAngle = 1.608;
    public static final double kL4CoralDoubleAngle = 0.698;

    public static final double kProcessorAngle = 2.727;
    public static final double kAlgaeOutAngle = 0.3 * 2 * Math.PI;
    public static final double kL2AlgaeAngle = 2.746;
    public static final double kL3AlgaeAngle = 2.672;

    public static final double kL2AlgaeHeight = 6.1;
    public static final double kL3AlgaeHeight = 11.2;
    public static final double kBargeHeight = 20;
  }
}
