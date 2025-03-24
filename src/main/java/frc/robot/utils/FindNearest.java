// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.DriveConstants;

/** Finds the nearest scoring location on the reef */
public class FindNearest {

  // Scoring locations for the blue alliance
  public static final Pose2d[] blueScoringLocations = {
      new Pose2d(new Translation2d(5.753, 3.861), Rotation2d.fromDegrees(180)),
      new Pose2d(new Translation2d(5.264, 3.015), Rotation2d.fromDegrees(120)),
      new Pose2d(new Translation2d(4.978, 2.85), Rotation2d.fromDegrees(120)),
      new Pose2d(new Translation2d(4.001, 2.85), Rotation2d.fromDegrees(60)),
      new Pose2d(new Translation2d(3.715, 3.015), Rotation2d.fromDegrees(60)),
      new Pose2d(new Translation2d(3.226, 3.861), Rotation2d.fromDegrees(0)),
      new Pose2d(new Translation2d(3.226, 4.191), Rotation2d.fromDegrees(0)),
      new Pose2d(new Translation2d(3.715, 5.037), Rotation2d.fromDegrees(-60)),
      new Pose2d(new Translation2d(4.001, 5.202), Rotation2d.fromDegrees(-60)),
      new Pose2d(new Translation2d(4.978, 5.202), Rotation2d.fromDegrees(-120)),
      new Pose2d(new Translation2d(5.264, 5.037), Rotation2d.fromDegrees(-120)),
      new Pose2d(new Translation2d(5.753, 4.191), Rotation2d.fromDegrees(180))
  };

  // Scoring locations for the red alliance
  public static final Pose2d[] redScoringLocations = new Pose2d[blueScoringLocations.length];
  static {
    for (int i = 0; i < blueScoringLocations.length; i++) {
      redScoringLocations[i] = AllianceFlipUtil.apply(blueScoringLocations[i]);
    }
  }

  public static final Pose2d[] blueSources = {
      new Pose2d(new Translation2d(1.7, 0.65), Rotation2d.fromDegrees(-127.5 + 180)),
      new Pose2d(new Translation2d(1.7, 7.38), Rotation2d.fromDegrees(127.5 - 180))
  };

  public static final Pose2d[] redSources = new Pose2d[blueSources.length];
  static {
    for (int i = 0; i < blueSources.length; i++) {
      redSources[i] = AllianceFlipUtil.apply(blueSources[i]);
    }
  }

  public static Pose2d getNearestScoringLocation(Pose2d currentPose) {
    Pose2d[] scoringLocations = AllianceFlipUtil.shouldFlip() ? redScoringLocations : blueScoringLocations;
    Pose2d nearestLocation = null;
    double nearestDistance = Double.MAX_VALUE;

    for (Pose2d location : scoringLocations) {
      double distance = currentPose.getTranslation().getDistance(location.getTranslation());
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearestLocation = location;
      }
    }

    return nearestLocation;
  }

  public static Pose2d getNearestSource(Pose2d currentPose) {
    Pose2d[] sources = AllianceFlipUtil.shouldFlip() ? redSources : blueSources;
    Pose2d nearestSource = null;
    double nearestDistance = DriveConstants.kMaxAutoDistance;

    for (Pose2d source : sources) {
      double distance = currentPose.getTranslation().getDistance(source.getTranslation());
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearestSource = source;
      }
    }

    return nearestSource;
  }
}
