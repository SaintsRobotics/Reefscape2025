package frc.robot.utils;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;

public interface IEstimatorWrapper {
    public void setVisionMeasurementStdDevs(Vector<N3> visionStddevs);

     public Rotation2d getGyroAngle();

     public void update(SwerveModulePosition[] swerveModulePositions);

     public Pose2d getEstimatedPosition();

     public double getGyroRate();

     public void addVisionMeasurement(Pose2d pose, double timestamp);

     public void resetPosition(SwerveModulePosition[] swerveModulePositions, Pose2d pose);

     public void resetGyro();

     public default void updateInternal(SwerveDriveKinematics kinematics, SwerveModulePosition[] swerveModulePositions, SwerveModuleState[] desiredStates, double deltatime) {}

     public default void resetAbsolutePosition(SwerveModulePosition[] swerveModulePositions, Pose2d pose) {}

      public default Pose2d getAbsolutePosition() {
        return new Pose2d();
      }
}