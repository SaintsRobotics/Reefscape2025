package frc.robot.utils;

import com.studica.frc.AHRS;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

public class RealEstimator implements IEstimatorWrapper {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final AHRS m_gyro;

    public RealEstimator(SwerveDriveKinematics kinematics, SwerveModulePosition[] swerveModulePositions, AHRS ahrs) {
      m_gyro = ahrs;

      m_poseEstimator = new SwerveDrivePoseEstimator(kinematics,
      getGyroAngle(), swerveModulePositions, new Pose2d(), VisionConstants.kOdometrySTDDevs,
      VisionConstants.kVisionSTDDevs);
    }

    public void setVisionMeasurementStdDevs(Vector<N3> visionStddevs) {
      m_poseEstimator.setVisionMeasurementStdDevs(visionStddevs);
    }

    public Rotation2d getGyroAngle() {
        return m_gyro.getRotation2d();
    }

    public void update(SwerveModulePosition[] swerveModulePositions) {
      m_poseEstimator.update(getGyroAngle(), swerveModulePositions);
    }

    public Pose2d getEstimatedPosition() {
      return m_poseEstimator.getEstimatedPosition();
    }

    public double getGyroRate() {
      return m_gyro.getRate();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
      m_poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void resetPosition(SwerveModulePosition[] swerveModulePositions, Pose2d pose) {
      m_poseEstimator.resetPosition(getGyroAngle(), swerveModulePositions, pose);
    }

    public void resetGyro() {
      m_gyro.reset();
    }
}