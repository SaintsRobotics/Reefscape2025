package frc.robot.constants;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

public final record Constants() {
	private static Robots robot = null;
	//private static Robots robot = Robots.COMPETITION;

	private enum Robots {
		COMPETITION,
		TLJR
	}

	private static final class RobotNameNotFound extends ExceptionInInitializerError {
		public RobotNameNotFound(String s) {
			super(s);
		}
	}

	private static void setRobot() {
		// First detect robot using roborio
		if (Robot.isSimulation()) {
			robot = Robots.COMPETITION;
		} else {
			BufferedReader reader = null;
			String line;
			try {
				try {
					reader = new BufferedReader(new FileReader("/etc/machine-info"));
					while ((line = reader.readLine()) != null) {
						if (line.contains("driftwood")) {
							robot = Robots.TLJR;
							break;
						}
					}
					if (robot == null) {
						throw new RobotNameNotFound("Robot name not found in /etc/machine-info");
					}
				} catch (FileNotFoundException e) {
						throw new RobotNameNotFound("Robot name not found in /etc/machine-info");
				} finally {
					if (reader != null) {
						reader.close();
					}
				}
			} catch (IOException e) {
				DriverStation.reportError("Error: Something went fatally wrong when detecting robot", e.getStackTrace());
			}
		}
	}

	static {
		if (robot == null) {
			setRobot();
		}

		switch (robot) {
			default:
			case TLJR:
				kFastPeriodicPeriod = TLJRConstants.kFastPeriodicPeriod;
				break;
		}
	}

	public static final double kFastPeriodicPeriod;

	/**
	 * Input/Output constants
	 */
	public static final record IOConstants() {
		static {
			if (robot == null) {
				setRobot();
			}

			switch (robot) {
				default:
				case TLJR:
					kDriverControllerPort = TLJRConstants.IOConstants.kDriverControllerPort;
					kOperatorControllerPort = TLJRConstants.IOConstants.kOperatorControllerPort;
					kControllerDeadband = TLJRConstants.IOConstants.kControllerDeadband;
					kSlowModeScalar = TLJRConstants.IOConstants.kSlowModeScalar;
					kDPadUp = TLJRConstants.IOConstants.kDPadUp;
					kDPadRight = TLJRConstants.IOConstants.kDPadRight;
					kDPadDown = TLJRConstants.IOConstants.kDPadDown;
					kDPadLeft = TLJRConstants.IOConstants.kDPadLeft;
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
			if (robot == null) {
				setRobot();
			}

			switch (robot) {
				default:
				case TLJR:
					kFrontLeftDriveMotorPort = TLJRConstants.DriveConstants.kFrontLeftDriveMotorPort;
					kRearLeftDriveMotorPort = TLJRConstants.DriveConstants.kRearLeftDriveMotorPort;
					kFrontRightDriveMotorPort = TLJRConstants.DriveConstants.kFrontRightDriveMotorPort;
					kRearRightDriveMotorPort = TLJRConstants.DriveConstants.kRearRightDriveMotorPort;

					kFrontLeftTurningMotorPort = TLJRConstants.DriveConstants.kFrontLeftTurningMotorPort;
					kRearLeftTurningMotorPort = TLJRConstants.DriveConstants.kRearLeftTurningMotorPort;
					kFrontRightTurningMotorPort = TLJRConstants.DriveConstants.kFrontRightTurningMotorPort;
					kRearRightTurningMotorPort = TLJRConstants.DriveConstants.kRearRightTurningMotorPort;

					kFrontLeftTurningEncoderPort = TLJRConstants.DriveConstants.kFrontLeftTurningEncoderPort;
					kRearLeftTurningEncoderPort = TLJRConstants.DriveConstants.kRearLeftTurningEncoderPort;
					kFrontRightTurningEncoderPort = TLJRConstants.DriveConstants.kFrontRightTurningEncoderPort;
					kRearRightTurningEncoderPort = TLJRConstants.DriveConstants.kRearRightTurningEncoderPort;

					kFrontLeftDriveMotorReversed = TLJRConstants.DriveConstants.kFrontLeftDriveMotorReversed;
					kRearLeftDriveMotorReversed = TLJRConstants.DriveConstants.kRearLeftDriveMotorReversed;
					kFrontRightDriveMotorReversed = TLJRConstants.DriveConstants.kFrontRightDriveMotorReversed;
					kRearRightDriveMotorReversed = TLJRConstants.DriveConstants.kRearRightDriveMotorReversed;

					kTrackWidth = TLJRConstants.DriveConstants.kTrackWidth;
					kWheelBase = TLJRConstants.DriveConstants.kWheelBase;
					kWheelDiameterMeters = TLJRConstants.DriveConstants.kWheelDiameterMeters;
					kDrivingGearRatio = TLJRConstants.DriveConstants.kDrivingGearRatio;

					kPModuleTurningController = TLJRConstants.DriveConstants.kPModuleTurningController;
					kDriveKinematics = TLJRConstants.DriveConstants.kDriveKinematics;

					kMaxSpeedMetersPerSecond = TLJRConstants.DriveConstants.kMaxSpeedMetersPerSecond;
					kMaxAngularSpeedRadiansPerSecond = TLJRConstants.DriveConstants.kMaxAngularSpeedRadiansPerSecond;

					kHeadingCorrectionTurningStopTime = TLJRConstants.DriveConstants.kHeadingCorrectionTurningStopTime;
					kPHeadingCorrectionController = TLJRConstants.DriveConstants.kPHeadingCorrectionController;
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
			if (robot == null) {
				setRobot();
			}

			switch (robot) {
				default:
				case TLJR:
					kCamPos = TLJRConstants.VisionConstants.kCamPos;
					kLimelightName = TLJRConstants.VisionConstants.kLimelightName;
					kIMUMode = TLJRConstants.VisionConstants.kIMUMode;

					kOdometrySTDDevs = TLJRConstants.VisionConstants.kOdometrySTDDevs;
					kVisionSTDDevs = TLJRConstants.VisionConstants.kVisionSTDDevs;

					kUseVision = TLJRConstants.VisionConstants.kUseVision;
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
			if (robot == null) {
				setRobot();
			}

			switch (robot) {
				default:
				case TLJR:
					kElevatorMotorPort = TLJRConstants.ElevatorConstants.kElevatorMotorPort;
					kElevatorCANrangePort = TLJRConstants.ElevatorConstants.kElevatorCANrangePort;

					kPElevator = TLJRConstants.ElevatorConstants.kPElevator;

					kElevatorGearing = TLJRConstants.ElevatorConstants.kElevatorGearing;
					kElevatorMaxSpeed = TLJRConstants.ElevatorConstants.kElevatorMaxSpeed;
					kElevatorFeedForward = TLJRConstants.ElevatorConstants.kElevatorFeedForward;
					kElevatorSpeedScalar = TLJRConstants.ElevatorConstants.kElevatorSpeedScalar;
					kElevatorBottom = TLJRConstants.ElevatorConstants.kElevatorBottom;
					kElevatorTop = TLJRConstants.ElevatorConstants.kElevatorTop;
					kElevatorDistanceThreshold = TLJRConstants.ElevatorConstants.kElevatorDistanceThreshold;

					kL1Height = TLJRConstants.ElevatorConstants.kL1Height;
					kL2Height = TLJRConstants.ElevatorConstants.kL2Height;
					kL3Height = TLJRConstants.ElevatorConstants.kL3Height;
					kL4Height = TLJRConstants.ElevatorConstants.kL4Height;
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
