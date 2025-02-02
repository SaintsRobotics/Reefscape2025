// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  ArrayList<Consumer<Pose3d>> m_consumers = new ArrayList<>();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    LimelightHelpers.setCameraPose_RobotSpace(VisionConstants.kLimelightName, VisionConstants.kF, VisionConstants.kS, VisionConstants.kU, VisionConstants.kR, VisionConstants.kP, VisionConstants.kY);
  }

  @Override
  public void periodic() {
    Pose3d pose = getPose();
    m_consumers.forEach((consumer) -> {
      consumer.accept(pose);
    });
  }

  public void addSubscriber(Consumer<Pose3d> consumer) {
    m_consumers.add(consumer);
  }

  public Pose3d getPose() {
    return LimelightHelpers.getBotPose3d(VisionConstants.kLimelightName);
  }
}
