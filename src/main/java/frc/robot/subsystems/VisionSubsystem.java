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

  private double m_dy;
  private double m_dp;
  private double m_dr;
  private double m_dt;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    LimelightHelpers.setCameraPose_RobotSpace(VisionConstants.kLimelightName, VisionConstants.kF, VisionConstants.kS,
        VisionConstants.kU, VisionConstants.kR, VisionConstants.kP, VisionConstants.kY);
    LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, VisionConstants.kIMUType_sync);

    m_dy = m_dp = m_dr = m_dt = 0;
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
    // calculate deltas
    m_dt = ahrs.getLastSensorTimestamp() - m_dt;
    if (m_dt > VisionConstants.kdtThreshold) {
      m_dy = m_dp = m_dr = m_dt = 0;
    } else {
      m_dy = MathUtil.inputModulus(ahrs.getYaw() - m_dy, -180, 180) / m_dt;
      m_dp = MathUtil.inputModulus(ahrs.getPitch() - m_dp, -180, 180) / m_dt;
      m_dr = MathUtil.inputModulus(ahrs.getRoll() - m_dr, -180, 180) / m_dt;
    }

    // calculate pose
    LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightName,
        estimator.getEstimatedPosition().getRotation().getDegrees(), m_dy, ahrs.getPitch(), m_dp, ahrs.getRoll(), m_dr);
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName); // WPILib
                                                                                                         // Ecosystem
                                                                                                         // (including
                                                                                                         // pathplanner)
                                                                                                         // assumes
                                                                                                         // origin on
                                                                                                         // blue side

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