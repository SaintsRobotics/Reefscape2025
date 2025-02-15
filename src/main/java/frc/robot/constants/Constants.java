package frc.robot.constants;

import java.lang.reflect.Field;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;

public record Constants() {
	private static Robots robot = Robots.DRIFTWOOD;

	private enum Robots {
		DRIFTWOOD
	}

	static {
		switch (robot) {
			default:
			case DRIFTWOOD:
				for (Field field : DriftwoodConstants.class.getDeclaredFields()) {
					field.setAccessible(true);
					try {
						Constants.class.getField(field.getName()).set(null, field.get(null));
					} catch (IllegalAccessException | NoSuchFieldException e) {
						throw new ExceptionInInitializerError(e);
					}
				}
				break;
		}
	}

	public static double kFastPeriodicPeriod;

	/**
	 * Input/Output constants
	 */
	public final static record IOConstants() {
		static {
			switch (robot) {
				default:
				case DRIFTWOOD:
					for (Field field : DriftwoodConstants.IOConstants.class.getDeclaredFields()) {
						field.setAccessible(true);
						try {
							IOConstants.class.getField(field.getName()).set(null, field.get(null));
						} catch (IllegalAccessException | NoSuchFieldException e) {
							throw new ExceptionInInitializerError(e);
						}
					}
					break;
			}
		}

		public static int kDriverControllerPort;
		public static int kOperatorControllerPort;

		public static double kControllerDeadband;
		public static double kSlowModeScalar;

		public static int kDPadUp;
		public static int kDPadRight;
		public static int kDPadDown;
		public static int kDPadLeft;
	}

	public static record DriveConstants() {
		static {
			switch (robot) {
				default:
				case DRIFTWOOD:
					for (Field field : DriftwoodConstants.DriveConstants.class.getDeclaredFields()) {
						field.setAccessible(true);
						try {
							DriveConstants.class.getField(field.getName()).set(null, field.get(null));
						} catch (IllegalAccessException | NoSuchFieldException e) {
							throw new ExceptionInInitializerError(e);
						}
					}
					break;
			}
		}

		public static int kFrontLeftDriveMotorPort;
		public static int kRearLeftDriveMotorPort;
		public static int kFrontRightDriveMotorPort;
		public static int kRearRightDriveMotorPort;

		public static int kFrontLeftTurningMotorPort;
		public static int kRearLeftTurningMotorPort;
		public static int kFrontRightTurningMotorPort;
		public static int kRearRightTurningMotorPort;

		public static int kFrontLeftTurningEncoderPort;
		public static int kRearLeftTurningEncoderPort;
		public static int kFrontRightTurningEncoderPort;
		public static int kRearRightTurningEncoderPort;

		public static boolean kFrontLeftDriveMotorReversed;
		public static boolean kRearLeftDriveMotorReversed;
		public static boolean kFrontRightDriveMotorReversed;
		public static boolean kRearRightDriveMotorReversed;

		/** Distance between centers of right and left wheels on robot (in meters). */
		public static double kTrackWidth;

		/** Distance between front and back wheels on robot (in meters). */
		public static double kWheelBase;

		/** Diameter of each wheel in the SDS MK4i swerve module (in meters) */
		public static double kWheelDiameterMeters;

		/** Gear ratio between the motor and the wheel. */
		public static double kDrivingGearRatio;

		public static double kPModuleTurningController;

		public static SwerveDriveKinematics kDriveKinematics;

		/** For a a SDS Mk4i L1 swerve base with Neos */
		public static double kMaxSpeedMetersPerSecond;
		/** For a a SDS Mk4i L1 swerve base with Neos */
		public static double kMaxAngularSpeedRadiansPerSecond;

		/** Heading Correction */
		public static double kHeadingCorrectionTurningStopTime;
		public static double kPHeadingCorrectionController;
	}

	public static record VisionConstants() {
		static {
			switch (robot) {
				default:
				case DRIFTWOOD:
					for (Field field : DriftwoodConstants.VisionConstants.class.getDeclaredFields()) {
						field.setAccessible(true);
						try {
							VisionConstants.class.getField(field.getName()).set(null, field.get(null));
						} catch (IllegalAccessException | NoSuchFieldException e) {
							throw new ExceptionInInitializerError(e);
						}
					}
					break;
			}
		}

		public static Pose3d kCamPos;

		public static String kLimelightName;

		// https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
		public static int kIMUMode;

		public static Vector<N3> kOdometrySTDDevs;
		public static Vector<N3> kVisionSTDDevs;

		public static boolean kUseVision;
	}

	public static record ElevatorConstants() {
		static {
			switch (robot) {
				default:
				case DRIFTWOOD:
					for (Field field : DriftwoodConstants.ElevatorConstants.class.getDeclaredFields()) {
						field.setAccessible(true);
						try {
							ElevatorConstants.class.getField(field.getName()).set(null, field.get(null));
						} catch (IllegalAccessException | NoSuchFieldException e) {
							throw new ExceptionInInitializerError(e);
						}
					}
					break;
			}
		}

		public static int kElevatorMotorPort;
		public static int kElevatorCANrangePort;

		public static double kPElevator;

		public static double kElevatorGearing;
		public static double kElevatorMaxSpeed;
		public static double kElevatorFeedForward;
		public static double kElevatorSpeedScalar;
		public static double kElevatorBottom;
		public static double kElevatorTop;
		public static double kElevatorDistanceThreshold;

		public static double kL1Height;
		public static double kL2Height;
		public static double kL3Height;
		public static double kL4Height;
	}
}
