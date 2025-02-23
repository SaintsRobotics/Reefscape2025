package frc.robot.utils;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Constants.VisionConstants;

public class SimulatedEstimator implements IEstimatorWrapper{
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final SwerveDriveOdometry m_absoluteOdometry;
    private final SwerveDriveKinematics m_kinematics;

    private double m_simulatedGyro = 0;
    private double m_prevGyro = 0;

    private double m_prevTime;

    private SwerveModulePosition[] m_prevPositions;

    private final boolean m_addNoise;

    public SimulatedEstimator(SwerveDriveKinematics kinematics, SwerveModulePosition[] swerveModulePositions) {
      this(kinematics, swerveModulePositions, true);
    }

    public SimulatedEstimator(SwerveDriveKinematics kinematics, SwerveModulePosition[] swerveModulePositions, boolean addNoise) {
      m_prevTime = Timer.getFPGATimestamp();
      m_prevPositions = swerveModulePositions;

      m_kinematics = kinematics;

      m_poseEstimator = new SwerveDrivePoseEstimator(kinematics,
      getGyroAngle(), swerveModulePositions, new Pose2d(), VisionConstants.kOdometrySTDDevs,
      VisionConstants.kVisionSTDDevs);

      m_absoluteOdometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), swerveModulePositions);

      m_addNoise = addNoise;
    }

    public void setVisionMeasurementStdDevs(Vector<N3> visionStddevs) {
      m_poseEstimator.setVisionMeasurementStdDevs(visionStddevs);
    }

    public Rotation2d getGyroAngle() {
      return new Rotation2d(m_simulatedGyro);
    }

    public void update(SwerveModulePosition[] swerveModulePositions, SwerveModuleState[] desiredStates) {
      m_prevGyro = m_simulatedGyro;

      final double timestamp = Timer.getFPGATimestamp();
      final double dt = timestamp - m_prevTime;

      m_simulatedGyro += m_kinematics.toChassisSpeeds(desiredStates).omegaRadiansPerSecond
        * (m_prevTime - timestamp);

      if (m_addNoise) {
				final double randomA = SimulationConstants.kRandom.nextDouble(-1, 1) * SimulationConstants.kMaxAngleError;
				final double randomD = SimulationConstants.kRandom.nextDouble(-1, 1) * SimulationConstants.kMaxDistanceError;
	
				for (int i = 0; i < 4; i++) {
					final double dA = swerveModulePositions[i].angle.getRadians() - m_prevPositions[i].angle.getRadians();
					final double dD = swerveModulePositions[i].distanceMeters - m_prevPositions[i].distanceMeters;
	
					swerveModulePositions[i].angle = new Rotation2d(swerveModulePositions[i].angle.getRadians() + (randomA * dA / dt));
					swerveModulePositions[i].distanceMeters += randomD * dD / dt;
				}
			}

      m_poseEstimator.update(getGyroAngle(), swerveModulePositions);

      m_absoluteOdometry.update(getGyroAngle(), swerveModulePositions);

      m_prevTime = timestamp;
      m_prevPositions = swerveModulePositions;
    }

    public Pose2d getEstimatedPosition() {
      return m_poseEstimator.getEstimatedPosition();
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
}