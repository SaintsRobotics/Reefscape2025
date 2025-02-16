package frc.robot.utils;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;

public interface IEstimatorWrapper {
    public void setVisionMeasurementStdDevs(Vector<N3> visionStddevs);

     public Rotation2d getGyroAngle();

     public void update(SwerveModulePosition[] swerveModulePositions, SwerveModuleState[] desiredStates);

     public Pose2d getEstimatedPosition();

     public double getGyroRate();

     public void addVisionMeasurement(Pose2d pose, double timestamp);

     /**
      * Resets the odometry position
      * @param swerveModulePositions
      * @param pose
      */
     public void resetPosition(SwerveModulePosition[] swerveModulePositions, Pose2d pose);

     /**
      * Resets the gyroscope
      */
     public void resetGyro();

     /**
      * Resets the true position of the robot.
      * Should only be used by simulation support code that need the true position of the robot.
      * @param swerveModulePositions The current swerve module positions
      * @param pose The pose to reset the robot to
      */
     public default void resetTruePosition(SwerveModulePosition[] swerveModulePositions, Pose2d pose) {}

     /**
      * Gets the true pose of the robot.
      * Should only be used by simulation support code that need the true position of the robot.
      * @return The robot's true pose
      */
      public default Pose2d getTruePosition() {
        return new Pose2d();
      }
}