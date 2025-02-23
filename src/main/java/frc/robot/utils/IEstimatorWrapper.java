package frc.robot.utils;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;

public interface IEstimatorWrapper {

	/**
	 * Sets the pose estimators trust for vision measurements. @see {@link edu.wpi.first.math.estimator.PoseEstimator#setVisionMeasurementStdDevs(edu.wpi.first.math.Matrix)}
	 * @param visionStddevs The standard deviation of all vision measurements. Higher values decrease trust
	 */
    public void setVisionMeasurementStdDevs(Vector<N3> visionStddevs);


	/**
	 * Gets the current gyroscope angle
	 * @return The gyroscope angle as a Rotation2d
	 */
     public Rotation2d getGyroAngle();

	 /**
	  * Updates the pose estimator using kinematics. @see {@link edu.wpi.first.math.estimator.PoseEstimator#update(Rotation2d, Object)}
	  * @param swerveModulePositions The current positions of the swerve modules
	  * @param desiredStates The desired states of the swerve modules.
	  */
     public void update(SwerveModulePosition[] swerveModulePositions, SwerveModuleState[] desiredStates);

	 /**
	  * Gets the estimated position of the robot. @see {@link edu.wpi.first.math.estimator.PoseEstimator#getEstimatedPosition()}
	  * @return The pose of the robot in meters
	  */
     public Pose2d getEstimatedPosition();

	 /**
	  * Gets the rate of the robot's yaw
	  * @return The robot's yaw rate in radians per second
	  */
     public double getGyroRate();

	 /**
	  * Adds a vision measurement to the pose estimator. @see {@link edu.wpi.first.math.estimator.PoseEstimator#addVisionMeasurement(Pose2d, double)}
	  * @param pose The pose determined by vision
	  * @param timestamp The timestamp of the pose
	  */
     public void addVisionMeasurement(Pose2d pose, double timestamp);

     /**
      * Resets the odometry's position
      * @param swerveModulePositions The resetted states of the swerve modules
      * @param pose The resetted pose of the robot in meters. Rotation is ignored
      */
     public void resetPosition(SwerveModulePosition[] swerveModulePositions, Pose2d pose);

     /**
      * Resets the gyroscope
      */
     public void resetGyro();

     /**
      * Resets the true position of the robot.
      * Should only be used by simulation support code that needs the true position of the robot.
      * @param swerveModulePositions The current swerve module positions
      * @param pose The pose to reset the robot to
      */
     public default void resetTruePosition(SwerveModulePosition[] swerveModulePositions, Pose2d pose) {}

     /**
      * Gets the true pose of the robot.
      * Should only be used by simulation support code that needs the true position of the robot.
      * @return The robot's true pose
      */
      public default Pose2d getTruePosition() {
        return new Pose2d();
      }
}