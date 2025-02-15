// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.SimulatedEstimator;
import frc.robot.utils.IEstimatorWrapper;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.RealEstimator;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftDriveMotorReversed);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kRearLeftDriveMotorReversed);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightDriveMotorReversed);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kRearRightDriveMotorReversed);

  private final Timer m_headingCorrectionTimer = new Timer();
  private final PIDController m_headingCorrectionPID = new PIDController(DriveConstants.kPHeadingCorrectionController,
      0, 0);
  private SwerveModulePosition[] m_swerveModulePositions = new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
  };

  private final IEstimatorWrapper m_poseEstimator;


  private SwerveModuleState[] m_desiredStates;

  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    if (Robot.isReal()) {
      m_poseEstimator = new RealEstimator(DriveConstants.kDriveKinematics, m_swerveModulePositions, new AHRS(NavXComType.kMXP_SPI));
    }
    else {
      m_poseEstimator = new SimulatedEstimator(DriveConstants.kDriveKinematics, m_swerveModulePositions);
    }

    this.zeroHeading();
    this.resetOdometry(new Pose2d());
    SmartDashboard.putData("Field", m_field);
    m_headingCorrectionTimer.restart();
    m_headingCorrectionPID.enableContinuousInput(-Math.PI, Math.PI);

    // TODO: Set a custom crop window for improved performance (the bot only needs
    // to see april tags on the reef)
    m_poseEstimator.setVisionMeasurementStdDevs(VisionConstants.kVisionSTDDevs);

    if (VisionConstants.kUseVision && Robot.isReal()) {
      LimelightHelpers.setCameraPose_RobotSpace(
          VisionConstants.kLimelightName,
          VisionConstants.kCamPos.getX(),
          VisionConstants.kCamPos.getY(),
          VisionConstants.kCamPos.getZ(),
          VisionConstants.kCamPos.getRotation().getX(),
          VisionConstants.kCamPos.getRotation().getY(),
          VisionConstants.kCamPos.getRotation().getZ());
      LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, VisionConstants.kIMUMode);
    }

    m_desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_swerveModulePositions = new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };

    m_poseEstimator.update(
        m_swerveModulePositions);

    boolean limelightReal = LimelightHelpers.getLatency_Pipeline(VisionConstants.kLimelightName) != 0.0;
    
    if (VisionConstants.kUseVision) {
      // Update LimeLight with current robot orientation
      LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), Math.toDegrees(m_poseEstimator.getGyroRate()), 0.0, 0.0, 0.0, 0.0);

      // Get the pose estimate
      LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers
          .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName);

      // Add it to your pose estimator if it is a valid measurement
      if (limelightMeasurement != null && limelightMeasurement.tagCount != 0 && m_poseEstimator.getGyroRate() < 720) {
        m_poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds);
      }
    }

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("gyro angle", m_poseEstimator.getGyroAngle().getDegrees());
    SmartDashboard.putNumber("odometryX", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("odometryY", m_poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putBoolean("Limelight isreal", limelightReal);

    // AdvantageScope Logging
    // max speed = 1 (for ease of use in AdvantageScope)
    double[] logData = {
        m_frontLeft.getPosition().angle.getDegrees(), m_frontLeft.driveOutput,
        m_frontRight.getPosition().angle.getDegrees(), m_frontRight.driveOutput,
        m_rearLeft.getPosition().angle.getDegrees(), m_rearLeft.driveOutput,
        m_rearRight.getPosition().angle.getDegrees(), m_rearRight.driveOutput,
    };

    double[] logDataDesired = {
      m_desiredStates[0].angle.getDegrees(), m_desiredStates[0].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
      m_desiredStates[1].angle.getDegrees(), m_desiredStates[1].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
      m_desiredStates[2].angle.getDegrees(), m_desiredStates[2].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
      m_desiredStates[3].angle.getDegrees(), m_desiredStates[3].speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
    };

    SmartDashboard.putNumberArray("AdvantageScope Swerve Desired States", logDataDesired);
    SmartDashboard.putNumberArray("AdvantageScope Swerve States", logData);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public Pose2d getSimulatedPose() {
    return m_poseEstimator.getAbsolutePosition();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotation      Angular rotation speed of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
    // If we are rotating, reset the timer
    if (rotation != 0) {
      m_headingCorrectionTimer.reset();
    }

    /*
     * Heading correction helps maintain the same heading and
     * prevents rotational drive while our robot is translating
     * 
     * For heading correction we use a timer to ensure that we
     * lose all rotational momentum before saving the heading
     * that we want to maintain
     */

    // TODO: Test heading correction without timer
    // TODO: Test heading correction using gyro's rotational velocity (if it is 0
    // then set heading instead of timer)

    // Save our desired rotation to a variable we can add our heading correction
    // adjustments to
    double calculatedRotation = rotation;

    double currentAngle = MathUtil.angleModulus(m_poseEstimator.getGyroAngle().getRadians());

    // If we are not translating or if not enough time has passed since the last
    // time we rotated
    if ((xSpeed == 0 && ySpeed == 0)
        || m_headingCorrectionTimer.get() < DriveConstants.kHeadingCorrectionTurningStopTime) {
      // Update our desired angle
      m_headingCorrectionPID.setSetpoint(currentAngle);
    } else {
      // If we are translating or if we have not rotated for a long enough time
      // then maintain our desired angle
      calculatedRotation = m_headingCorrectionPID.calculate(currentAngle);
    }

    // Depending on whether the robot is being driven in field relative, calculate
    // the desired states for each of the modules
    m_desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, calculatedRotation,
                m_poseEstimator.getGyroAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, calculatedRotation));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void resetSimulatedOdometry(Pose2d pose) {
    m_poseEstimator.resetAbsolutePosition(
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_poseEstimator.resetGyro();
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(pose, timestamp);
  }

  /** Sets the module states every 10ms (100Hz), faster than the regular periodic loop */
  public void fastPeriodic() {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        m_desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(m_desiredStates[0]);
    m_frontRight.setDesiredState(m_desiredStates[1]);
    m_rearLeft.setDesiredState(m_desiredStates[2]);
    m_rearRight.setDesiredState(m_desiredStates[3]);

    m_poseEstimator.updateInternal(DriveConstants.kDriveKinematics, m_swerveModulePositions, m_desiredStates, Constants.kFastPeriodicPeriod);
  }
}