package frc.robot.utils;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Constants.VisionConstants;

public class SimulatedEstimator implements IEstimatorWrapper{
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDriveOdometry m_absoluteOdometry;

    private double m_simulatedGyro = 0;
    private double m_prevGyro = 0;

    private Transform2d m_jitter;

    public SimulatedEstimator(SwerveDriveKinematics kinematics, SwerveModulePosition[] swerveModulePositions) {
      m_jitter = new Transform2d();

      m_poseEstimator = new SwerveDrivePoseEstimator(kinematics,
      getGyroAngle(), swerveModulePositions, new Pose2d(), VisionConstants.kOdometrySTDDevs,
      VisionConstants.kVisionSTDDevs);

      m_absoluteOdometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), swerveModulePositions);
    }

    public void setVisionMeasurementStdDevs(Vector<N3> visionStddevs) {
      m_poseEstimator.setVisionMeasurementStdDevs(visionStddevs);
    }

    public Rotation2d getGyroAngle() {
      return new Rotation2d(m_simulatedGyro);
    }

    public void update(SwerveModulePosition[] swerveModulePositions) {
      final Pose2d initialPose = m_poseEstimator.getEstimatedPosition();

      m_poseEstimator.update(getGyroAngle(), swerveModulePositions);

      // add error
      final Transform2d dPose = m_poseEstimator.getEstimatedPosition().minus(initialPose);
      m_jitter = m_jitter.plus(new Transform2d(new Translation2d(SimulationConstants.kRandom.nextDouble() * SimulationConstants.kMaxTranslationError * dPose.getX(), SimulationConstants.kRandom.nextDouble() * SimulationConstants.kMaxTranslationError * dPose.getY()), new Rotation2d(SimulationConstants.kRandom.nextDouble() * SimulationConstants.kMaxRotationError * dPose.getRotation().getRadians())));

      m_absoluteOdometry.update(getGyroAngle(), swerveModulePositions);
    }

    public Pose2d getEstimatedPosition() {
      return m_poseEstimator.getEstimatedPosition().plus(m_jitter);
    }


    public Pose2d getAbsolutePosition() {
      return m_absoluteOdometry.getPoseMeters();
    }

    public double getGyroRate() {
      return (m_simulatedGyro - m_prevGyro) / Constants.kFastPeriodicPeriod;
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
      m_poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void resetPosition(SwerveModulePosition[] swerveModulePositions, Pose2d pose) {
      m_poseEstimator.resetPosition(getGyroAngle(), swerveModulePositions, new Pose2d(pose.getX(), pose.getY(), new Rotation2d(m_simulatedGyro)));
    }

    public void resetAbsolutePosition(SwerveModulePosition[] swerveModulePositions, Pose2d pose) {
      m_absoluteOdometry.resetPosition(getGyroAngle(), swerveModulePositions, new Pose2d(pose.getX(), pose.getY(), new Rotation2d(m_simulatedGyro)));
    }

    public void resetGyro() {
      m_simulatedGyro = 0;
      m_prevGyro = 0;
    }

    public void updateInternal(SwerveDriveKinematics kinematics, SwerveModulePosition[] swerveModulePositions, SwerveModuleState[] desiredStates, double deltatime) {
      m_prevGyro = m_simulatedGyro;

      m_simulatedGyro += kinematics.toChassisSpeeds(desiredStates).omegaRadiansPerSecond
        * deltatime;
    }
}