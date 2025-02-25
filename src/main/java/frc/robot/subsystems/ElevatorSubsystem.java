// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.Interlocks;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkFlex m_elevatorMotor;
  private final CANrange m_elevatorRange = new CANrange(ElevatorConstants.kElevatorCANrangePort);

  private final PIDController m_PIDController = new PIDController(ElevatorConstants.kPElevator, 0, 0, Constants.kFastPeriodicPeriod);

  private double m_targetPosition = 0;
  private double m_motorOffset = 0;

  private double m_elevatorMin;
  private double m_elevatorMax;

  private final Interlocks m_interlocks;

  public ElevatorSubsystem(Interlocks interlocks) {
    m_elevatorMin = Constants.ElevatorConstants.kElevatorBottom;
    m_elevatorMax = Constants.ElevatorConstants.kElevatorTop;

    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.encoder.positionConversionFactor(ElevatorConstants.kElevatorGearing);

    m_elevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorPort, MotorType.kBrushless);
    // TODO: set to reset and persist after testing
    m_elevatorMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_interlocks = interlocks;
  }

  @Override
  public void periodic() {
    m_interlocks.setElevatorHeight(m_elevatorMotor.getEncoder().getPosition() + m_motorOffset);

    // This method will be called once per scheduler run
    if (m_elevatorRange.getDistance().getValueAsDouble() < ElevatorConstants.kElevatorDistanceThreshold) {
      // This offset is set when the distance sensor detects that the elevator is at the bottom 
      // At the bottom, the motor's position + offset should equal 0
      m_motorOffset = -m_elevatorMotor.getEncoder().getPosition();
    }
  }

  public void fastPeriodic() {
    double output = m_PIDController.calculate(
      m_elevatorMotor.getEncoder().getPosition() + m_motorOffset,
      m_targetPosition) + ElevatorConstants.kElevatorFeedForward;
    output = MathUtil.clamp(output, -ElevatorConstants.kElevatorMaxSpeed, ElevatorConstants.kElevatorMaxSpeed);

    m_elevatorMotor.set(m_interlocks.clampElevatorMotorSet(output));

    SmartDashboard.putNumber("elevator motor output", output);
  }

  public void joystickMovement(double joystickY) {
    // Moves the target position by joystickY multiplied by the constant kSpeed, clamped between the top and bottom heights
    m_targetPosition += joystickY * ElevatorConstants.kElevatorSpeedScalar * Robot.kDefaultPeriod;
    m_targetPosition = MathUtil.clamp(m_targetPosition, m_elevatorMin, m_elevatorMax);
    // Display this number for now so we can see it
    SmartDashboard.putNumber("elevator targetPosition", m_targetPosition);
  }

  public void setHeight(double level) {
    // Set the elevator target height to the corresponding level (L1, L2, L3, L4)
    m_targetPosition = level;
  }

  public double getHeightSetpoint() {
    return m_targetPosition;
  }

  public double getCurrentHeight() {
    return m_elevatorMotor.getEncoder().getPosition() + m_motorOffset;
  }

  public boolean atSetpoint() {
    return m_PIDController.atSetpoint();
  }
}
