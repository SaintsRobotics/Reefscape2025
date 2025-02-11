package frc.robot.utils;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
}