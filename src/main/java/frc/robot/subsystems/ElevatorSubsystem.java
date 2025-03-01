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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.Interlocks;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkFlex m_elevatorMotor;
  // private final CANrange m_elevatorRange = new CANrange(ElevatorConstants.kElevatorCANrangePort);

  private final TrapezoidProfile.Constraints m_contraints = new TrapezoidProfile.Constraints(ElevatorConstants.kMaxV, ElevatorConstants.kMaxA);
  private final ProfiledPIDController m_PIDController = new ProfiledPIDController(ElevatorConstants.kPElevator, 0, 0, m_contraints, Constants.kFastPeriodicPeriod);

  private double m_motorOffset = 0;

  private double m_output = 0;

  private final Interlocks m_interlocks;

  private double m_speedOverride;

  public ElevatorSubsystem(Interlocks interlocks) {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.encoder.positionConversionFactor(ElevatorConstants.kElevatorGearing);
    motorConfig.idleMode(IdleMode.kBrake);

    m_elevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorPort, MotorType.kBrushless);
    // TODO: set to reset and persist after testing
    m_elevatorMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);


    m_PIDController.setTolerance(ElevatorConstants.kPositionTolerance, ElevatorConstants.kVelocityTolerance);
    m_interlocks = interlocks;
  }

  @Override
  public void periodic() {
    m_interlocks.setElevatorHeight(m_elevatorMotor.getEncoder().getPosition() + m_motorOffset);

    /*
     * The order of callbacks is as follows:
     *  The timed robot periodic will run
     *    Then the command command scheduler will run
     *      Then all periodics will run
     *      Then all commands will run
     *    Then the fast periodics will run
     *    Then the fast periodics will run again
     * 
     * This means that we will set overrideSpeed to 0 in each periodic
     *  Then a command might cause this to become non zero
     *  In that case, the two fast periodics will use the speed override instead of the setpoint
     */

    m_speedOverride = 0;

    /*
     * if (m_elevatorRange.getDistance().getValueAsDouble() <
     * ElevatorConstants.kElevatorDistanceThreshold) {
     * // This offset is set when the distance sensor detects that the elevator is
     * at the bottom
     * // At the bottom, the motor's position + offset should equal 0
     * zeroPosition();
     * }
     */

    SmartDashboard.putNumber("Elevator Height", getCurrentHeight());
    SmartDashboard.putNumber("Elevator Setpoint", m_PIDController.getGoal().position);
    SmartDashboard.putNumber("Elevator output", m_output);
  }

  public void fastPeriodic() {
    m_output = m_PIDController.calculate(
        getCurrentHeight()) + ElevatorConstants.kElevatorFeedForward;
    m_output = m_speedOverride != 0 ? m_speedOverride + ElevatorConstants.kElevatorFeedForward : m_output;

    m_elevatorMotor.set(m_interlocks.clampElevatorMotorSet(m_output));
  }

  public void setHeight(double level) {
    // Set the elevator target height to the corresponding level (L1, L2, L3, L4)
    m_PIDController.setGoal(level); //TODO: clamp setpoint
  }

  public double getHeightSetpoint() {
    return m_PIDController.getGoal().position;
  }

  public double getCurrentHeight() {
    return m_elevatorMotor.getEncoder().getPosition() + m_motorOffset;
  }

  public boolean atSetpoint() {
    return m_PIDController.atGoal();
  }

  /**
   * Sets the speed of the elevator and overwrites the setpoint until the next period
   * @param speed The speed without the feedforwards
   */
  public void setSpeed(double speed) {
    m_speedOverride = speed;
    m_PIDController.reset(getCurrentHeight());
  }

  /**
   * Zeroes the position so that the current elevator position is zero
   */
  public void zeroPosition() {
    zeroPosition(0);
  }


  /**
   * Zereos the position so that the current elevator position is offset
   * @param offset
   */
  public void zeroPosition(double offset) {
    m_motorOffset = -m_elevatorMotor.getEncoder().getPosition() + offset;
    setHeight(offset);
    m_PIDController.reset(offset);
  }
}
