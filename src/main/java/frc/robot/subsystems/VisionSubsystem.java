// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
  ArrayList<Pair<SwerveDrivePoseEstimator, AHRS>> m_estimators = new ArrayList<>();

  // Rate: Change over time
  private double m_yawRate;
  private double m_pitchRate;
  private double m_rollRate;
  private double m_deltaTime;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    LimelightHelpers.setCameraPose_RobotSpace(
      VisionConstants.kLimelightName, 
      VisionConstants.kCamPos.getX(),
      VisionConstants.kCamPos.getY(),
      VisionConstants.kCamPos.getZ(), 
      VisionConstants.kCamPos.getRotation().getX(), 
      VisionConstants.kCamPos.getRotation().getY(), 
      VisionConstants.kCamPos.getRotation().getZ()
    );
    LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, VisionConstants.kIMUType_sync);

    m_yawRate = m_pitchRate = m_rollRate = m_deltaTime = 0;
  }

  @Override
  public void periodic() {
    m_estimators.forEach((Pair<SwerveDrivePoseEstimator, AHRS> estimator) -> {
      estimatePose(estimator.getFirst(), estimator.getSecond());
    });
  }

  public void addSubscriber(Pair<SwerveDrivePoseEstimator, AHRS> estimator) {
    m_estimators.add(estimator);
  }

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, mode);
  }

  private void estimatePose(SwerveDrivePoseEstimator estimator, AHRS ahrs) {
    // calculate deltas and rates of rotation
    m_deltaTime = ahrs.getLastSensorTimestamp() - m_deltaTime;
    if (m_deltaTime > VisionConstants.kdtThreshold) {
      m_yawRate = m_pitchRate = m_rollRate = m_deltaTime = 0;
    } else {
      m_yawRate = MathUtil.inputModulus(ahrs.getYaw() - m_yawRate, -180, 180) / m_deltaTime;
      m_pitchRate = MathUtil.inputModulus(ahrs.getPitch() - m_pitchRate, -180, 180) / m_deltaTime;
      m_rollRate = MathUtil.inputModulus(ahrs.getRoll() - m_rollRate, -180, 180) / m_deltaTime;
    }

    // calculate pose
    // WPILIB assumes origin on blue side
    LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightName,
        estimator.getEstimatedPosition().getRotation().getDegrees(), 
        m_yawRate, 
        ahrs.getPitch(), 
        m_pitchRate, 
        ahrs.getRoll(), 
        m_rollRate
    );
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName); 

    if (estimate == null) {
      DriverStation.reportError("Bad limelight data: wrong packet size", false);
      return;
    }

    if (estimate.tagCount == 0) { // no estimate available
      return;
    }

    if (estimate.tagCount == 1 && estimate.rawFiducials.length == 1) { // single target, non megatag functionality
      // enforce higher confidence
      if (estimate.rawFiducials[0].ambiguity > VisionConstants.kMaxAmbiguityNonMega
          || estimate.rawFiducials[0].distToCamera > VisionConstants.kMaxDistNonMega) {
        return;
      }
    }

    estimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, VisionConstants.kVisionSTDDevs);
  }
}