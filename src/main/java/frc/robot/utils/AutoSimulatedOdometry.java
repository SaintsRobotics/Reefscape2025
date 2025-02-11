package frc.robot.utils;

import com.studica.frc.AHRS;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

public class AutoSimulatedOdometry {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDriveOdometry m_simulatedOdometry;
    private final AHRS m_gyro;

    private double m_simulatedGyro = 0;
    private double m_prevGyro = 0;

    public AutoSimulatedOdometry(SwerveDriveKinematics kinematics, SwerveModulePosition[] swerveModulePositions, AHRS ahrs) {
      m_gyro = ahrs;

      m_poseEstimator = new SwerveDrivePoseEstimator(kinematics,
      getGyroAngle(), swerveModulePositions, new Pose2d(), VisionConstants.kOdometrySTDDevs,
      VisionConstants.kVisionSTDDevs);

      m_simulatedOdometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), swerveModulePositions);
    }

    public void setVisionMeasurementStdDevs(Vector<N3> visionStddevs) {
      m_poseEstimator.setVisionMeasurementStdDevs(visionStddevs);
    }

    public Rotation2d getGyroAngle() {
      if (Robot.isReal()) {
        return m_gyro.getRotation2d();
      }
      return new Rotation2d(m_simulatedGyro);
    }

    public void update(SwerveModulePosition[] swerveModulePositions) {
      m_poseEstimator.update(getGyroAngle(), swerveModulePositions);
      m_simulatedOdometry.update(getGyroAngle(), swerveModulePositions);
    }

    public Pose2d getEstimatedPosition() {
      if (Robot.isReal()) {
        return m_poseEstimator.getEstimatedPosition();
      }
      return m_simulatedOdometry.getPoseMeters(); //TODO add error
    }

    public Pose2d getSimulatedPosition() {
      if (Robot.isReal()) {
        return new Pose2d(); // does not work for real robot
      }
      return m_simulatedOdometry.getPoseMeters();
    }

    public double getGyroRate() {
      if (Robot.isReal()) {
        return m_gyro.getRate();
      }
      return (m_simulatedGyro - m_prevGyro) / Constants.kFastPeriodicPeriod;
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

    public void simulate(SwerveDriveKinematics kinematics, SwerveModuleState[] desiredStates) {
      m_prevGyro = m_simulatedGyro;

      m_simulatedGyro += kinematics.toChassisSpeeds(desiredStates).omegaRadiansPerSecond
        * Constants.kFastPeriodicPeriod;
    }
}