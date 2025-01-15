// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final SparkMaxConfig m_driveMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_turningMotorConfig = new SparkMaxConfig();

  private final CANcoder m_turningEncoder;
  private final CANcoderConfigurator m_turningEncoderConfigurator;

  private final PIDController m_turningPIDController = new PIDController(DriveConstants.kPModuleTurningController, 0,
      0);

  private SwerveModuleState m_state = new SwerveModuleState();
  private double m_distance;

  public double driveOutput;
  public double turnOutput;

  /**
   * Constructs a {@link SwerveModule}.
   *
   * @param driveMotorPort       The port of the drive motor.
   * @param turningMotorPort     The port of the turning motor.
   * @param turningEncoderPort   The port of the turning encoder.
   * @param driveMotorReversed   Whether the drive motor is reversed.
   * @param turningEncoderOffset Offset of the turning encoder.
   */
  public SwerveModule(
      int driveMotorPort,
      int turningMotorPort,
      int turningEncoderPort,
      boolean driveMotorReversed,
      double turningEncoderOffset) {
    m_driveMotor = new SparkMax(driveMotorPort, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorPort, MotorType.kBrushless);
    m_turningEncoder = new CANcoder(turningEncoderPort);
    m_turningEncoderConfigurator = m_turningEncoder.getConfigurator();

    // converts default units to meters per second
    m_driveMotorConfig.encoder.positionConversionFactor(DriveConstants.kWheelDiameterMeters * Math.PI / 60 / DriveConstants.kDrivingGearRatio);
    m_driveMotorConfig.inverted(driveMotorReversed);

    m_turningMotorConfig.idleMode(IdleMode.kBrake);

    m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(m_turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // TODO: CANcoder offsets are now set on the device manually using Pheonix Tuner X
    m_turningEncoderConfigurator.apply(new MagnetSensorConfigs().withMagnetOffset(-turningEncoderOffset));

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    m_distance += m_state.speedMetersPerSecond * Robot.kDefaultPeriod;

    // If the robot is real, then return the swerve module state by reading from the
    // actual encoders
    // If the robot is simulated, then return the swerve module state using the
    // expected values
    return Robot.isReal()
        ? new SwerveModulePosition(m_driveMotor.getEncoder().getPosition(),
            getEncoderAngle(m_turningEncoder))
        : new SwerveModulePosition(m_distance, m_state.angle);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    m_state = desiredState;
    m_state.optimize(getEncoderAngle(m_turningEncoder));
    driveOutput = m_state.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;

    turnOutput = m_turningPIDController.calculate(getEncoderAngle(m_turningEncoder).getRadians(),
        m_state.angle.getRadians());

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /**
   * Returns the angle of a CANcoder
   * 
   * The CANcoder now gives values in rotations which is useless, so this method
   * translates the CANcoder output into a Rotation2D
   * 
   * @param encoder The encoder to get the absolute angle of.
   * @return A Rotation2d of the absolute angle.
   */
  public Rotation2d getEncoderAngle(CANcoder encoder) {
    return new Rotation2d(encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
  }
}
