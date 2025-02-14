package frc.robot.constants;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;

public final record Constants() {
	private static final Robots robot = Robots.DRIFTWOOD;

	private enum Robots {
		DRIFTWOOD
	}

	static {
		switch (robot) {
			default:
			case DRIFTWOOD:
				kFastPeriodicPeriod = DriftwoodConstants.kFastPeriodicPeriod;
				break;
		}
	}

	public static final double kFastPeriodicPeriod;

	/**
	 * Input/Output constants
	 */
	public static final record IOConstants() {
		static {
			switch (robot) {
				default:
				case DRIFTWOOD:
					kDriverControllerPort = DriftwoodConstants.IOConstants.kDriverControllerPort;
					kOperatorControllerPort = DriftwoodConstants.IOConstants.kOperatorControllerPort;
					kControllerDeadband = DriftwoodConstants.IOConstants.kControllerDeadband;
					kSlowModeScalar = DriftwoodConstants.IOConstants.kSlowModeScalar;
					kDPadUp = DriftwoodConstants.IOConstants.kDPadUp;
					kDPadRight = DriftwoodConstants.IOConstants.kDPadRight;
					kDPadDown = DriftwoodConstants.IOConstants.kDPadDown;
					kDPadLeft = DriftwoodConstants.IOConstants.kDPadLeft;
					break;
			}
		}

		public static final int kDriverControllerPort;
		public static final int kOperatorControllerPort;

		public static final double kControllerDeadband;
		public static final double kSlowModeScalar;

		public static final int kDPadUp;
		public static final int kDPadRight;
		public static final int kDPadDown;
		public static final int kDPadLeft;
	}

	public static final record DriveConstants() {
		static {
			switch (robot) {
				default:
				case DRIFTWOOD:
					kFrontLeftDriveMotorPort = DriftwoodConstants.DriveConstants.kFrontLeftDriveMotorPort;
					kRearLeftDriveMotorPort = DriftwoodConstants.DriveConstants.kRearLeftDriveMotorPort;
					kFrontRightDriveMotorPort = DriftwoodConstants.DriveConstants.kFrontRightDriveMotorPort;
					kRearRightDriveMotorPort = DriftwoodConstants.DriveConstants.kRearRightDriveMotorPort;
					
					kFrontLeftTurningMotorPort = DriftwoodConstants.DriveConstants.kFrontLeftTurningMotorPort;
					kRearLeftTurningMotorPort = DriftwoodConstants.DriveConstants.kRearLeftTurningMotorPort;
					kFrontRightTurningMotorPort = DriftwoodConstants.DriveConstants.kFrontRightTurningMotorPort;
					kRearRightTurningMotorPort = DriftwoodConstants.DriveConstants.kRearRightTurningMotorPort;

					kFrontLeftTurningEncoderPort = DriftwoodConstants.DriveConstants.kFrontLeftTurningEncoderPort;
					kRearLeftTurningEncoderPort = DriftwoodConstants.DriveConstants.kRearLeftTurningEncoderPort;
					kFrontRightTurningEncoderPort = DriftwoodConstants.DriveConstants.kFrontRightTurningEncoderPort;
					kRearRightTurningEncoderPort = DriftwoodConstants.DriveConstants.kRearRightTurningEncoderPort;

					kFrontLeftDriveMotorReversed = DriftwoodConstants.DriveConstants.kFrontLeftDriveMotorReversed;
					kRearLeftDriveMotorReversed = DriftwoodConstants.DriveConstants.kRearLeftDriveMotorReversed;
					kFrontRightDriveMotorReversed = DriftwoodConstants.DriveConstants.kFrontRightDriveMotorReversed;
					kRearRightDriveMotorReversed = DriftwoodConstants.DriveConstants.kRearRightDriveMotorReversed;

					kTrackWidth = DriftwoodConstants.DriveConstants.kTrackWidth;
					kWheelBase = DriftwoodConstants.DriveConstants.kWheelBase;
					kWheelDiameterMeters = DriftwoodConstants.DriveConstants.kWheelDiameterMeters;
					kDrivingGearRatio = DriftwoodConstants.DriveConstants.kDrivingGearRatio;

					kPModuleTurningController = DriftwoodConstants.DriveConstants.kPModuleTurningController;
					kDriveKinematics = DriftwoodConstants.DriveConstants.kDriveKinematics;

					kMaxSpeedMetersPerSecond = DriftwoodConstants.DriveConstants.kMaxSpeedMetersPerSecond;
					kMaxAngularSpeedRadiansPerSecond = DriftwoodConstants.DriveConstants.kMaxAngularSpeedRadiansPerSecond;

					kHeadingCorrectionTurningStopTime = DriftwoodConstants.DriveConstants.kHeadingCorrectionTurningStopTime;
					kPHeadingCorrectionController = DriftwoodConstants.DriveConstants.kPHeadingCorrectionController;
					break;
			}
		}

		public static final int kFrontLeftDriveMotorPort;
		public static final int kRearLeftDriveMotorPort;
		public static final int kFrontRightDriveMotorPort;
		public static final int kRearRightDriveMotorPort;

		public static final int kFrontLeftTurningMotorPort;
		public static final int kRearLeftTurningMotorPort;
		public static final int kFrontRightTurningMotorPort;
		public static final int kRearRightTurningMotorPort;

		public static final int kFrontLeftTurningEncoderPort;
		public static final int kRearLeftTurningEncoderPort;
		public static final int kFrontRightTurningEncoderPort;
		public static final int kRearRightTurningEncoderPort;

		public static final boolean kFrontLeftDriveMotorReversed;
		public static final boolean kRearLeftDriveMotorReversed;
		public static final boolean kFrontRightDriveMotorReversed;
		public static final boolean kRearRightDriveMotorReversed;

		/** Distance between centers of right and left wheels on robot (in meters). */
		public static final double kTrackWidth;

		/** Distance between front and back wheels on robot (in meters). */
		public static final double kWheelBase;

		/** Diameter of each wheel in the SDS MK4i swerve module (in meters) */
		public static final double kWheelDiameterMeters;

		/** Gear ratio between the motor and the wheel. */
		public static final double kDrivingGearRatio;

		public static final double kPModuleTurningController;

		public static final SwerveDriveKinematics kDriveKinematics;

		/** For a a SDS Mk4i L1 swerve base with Neos */
		public static final double kMaxSpeedMetersPerSecond;
		/** For a a SDS Mk4i L1 swerve base with Neos */
		public static final double kMaxAngularSpeedRadiansPerSecond;

		/** Heading Correction */
		public static final double kHeadingCorrectionTurningStopTime;
		public static final double kPHeadingCorrectionController;
	}

	public static final record VisionConstants() {
		static {
			switch (robot) {
				default:
				case DRIFTWOOD:
					kCamPos = DriftwoodConstants.VisionConstants.kCamPos;
					kLimelightName = DriftwoodConstants.VisionConstants.kLimelightName;
					kIMUMode = DriftwoodConstants.VisionConstants.kIMUMode;

					kOdometrySTDDevs = DriftwoodConstants.VisionConstants.kOdometrySTDDevs;
					kVisionSTDDevs = DriftwoodConstants.VisionConstants.kVisionSTDDevs;

					kUseVision = DriftwoodConstants.VisionConstants.kUseVision;
					break;
			}
		}

		public static final Pose3d kCamPos;

		public static final String kLimelightName;

		// https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
		public static final int kIMUMode;

		public static final Vector<N3> kOdometrySTDDevs;
		public static final Vector<N3> kVisionSTDDevs;

		public static final boolean kUseVision;
	}

	public static final record ElevatorConstants() {
		static {
			switch (robot) {
				default:
				case DRIFTWOOD:
					kElevatorMotorPort = DriftwoodConstants.ElevatorConstants.kElevatorMotorPort;
					kElevatorCANrangePort = DriftwoodConstants.ElevatorConstants.kElevatorCANrangePort;

					kPElevator = DriftwoodConstants.ElevatorConstants.kPElevator;

					kElevatorGearing = DriftwoodConstants.ElevatorConstants.kElevatorGearing;
					kElevatorMaxSpeed = DriftwoodConstants.ElevatorConstants.kElevatorMaxSpeed;
					kElevatorFeedForward = DriftwoodConstants.ElevatorConstants.kElevatorFeedForward;
					kElevatorSpeedScalar = DriftwoodConstants.ElevatorConstants.kElevatorSpeedScalar;
					kElevatorBottom = DriftwoodConstants.ElevatorConstants.kElevatorBottom;
					kElevatorTop = DriftwoodConstants.ElevatorConstants.kElevatorTop;
					kElevatorDistanceThreshold = DriftwoodConstants.ElevatorConstants.kElevatorDistanceThreshold;

					kL1Height = DriftwoodConstants.ElevatorConstants.kL1Height;
					kL2Height = DriftwoodConstants.ElevatorConstants.kL2Height;
					kL3Height = DriftwoodConstants.ElevatorConstants.kL3Height;
					kL4Height = DriftwoodConstants.ElevatorConstants.kL4Height;
					break;
			}
		}

		public static final int kElevatorMotorPort;
		public static final int kElevatorCANrangePort;

		public static final double kPElevator;

		public static final double kElevatorGearing;
		public static final double kElevatorMaxSpeed;
		public static final double kElevatorFeedForward;
		public static final double kElevatorSpeedScalar;
		public static final double kElevatorBottom;
		public static final double kElevatorTop;
		public static final double kElevatorDistanceThreshold;

		public static final double kL1Height;
		public static final double kL2Height;
		public static final double kL3Height;
		public static final double kL4Height;
	}
}
