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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.utils.Interlocks;

public class EndEffectorSubsystem extends SubsystemBase {
  private final SparkFlex m_pivotMotor;
  private final SparkFlex m_effectorMotor;
  private final CANrange m_endEffectorRange = new CANrange(EndEffectorConstants.kEndEffectorCANrangePort);

  private final PIDController m_PIDController = new PIDController(EndEffectorConstants.kPEndEffector, 0, 0, Constants.kFastPeriodicPeriod);

  private double targetRotation = 0;
  private double effectorOutput = 0;

  private double m_output;

  private final Interlocks m_interlocks;

  private double m_speedOverride;

  private double m_aggressiveComponent;

  // Pivoting controls: A is L1, B is L2 & L3, Y is L4 or use right joystick
  // Intake/Outtake controls: Right Bumper: Intake Algae, Left Bumper: Outtake Algae
  //                          Right Trigger: Intake Coral, Left Trigger Outtake Coral

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem(Interlocks interlocks) {
    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.absoluteEncoder.positionConversionFactor(Math.PI * 2); // convert to radians

    m_pivotMotor = new SparkFlex(EndEffectorConstants.kPivotMotorPort, MotorType.kBrushless);
    m_effectorMotor = new SparkFlex(EndEffectorConstants.kEffectorMotorPort, MotorType.kBrushless);

    // TODO: set to reset and persist after testing
    m_pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // TODO: maybe reverse effector motor
    m_PIDController.setTolerance(EndEffectorConstants.kPivotTolerance);

    m_interlocks = interlocks;
    m_aggressiveComponent = 0;
  }

  @Override
  public void periodic() {
    m_interlocks.setPivotPosition(getPivotPosition());

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

    SmartDashboard.putBoolean("Is Holding", isHolding());
    SmartDashboard.putNumber("Pivot Angle 2", getPivotPosition());
    SmartDashboard.putNumber("raw rotations", m_pivotMotor.getAbsoluteEncoder().getPosition() / Math.PI / 2.0);
    SmartDashboard.putNumber("Pivot Output", m_output);
    // This method will be called once per scheduler run
  }

  public void fastPeriodic(){
    m_output = -m_PIDController.calculate(getPivotPosition(), targetRotation + m_aggressiveComponent);
    m_output = m_speedOverride != 0 ? m_speedOverride : m_output;

    m_pivotMotor.set(m_interlocks.clampPivotMotorSet(m_output));
    m_effectorMotor.set(effectorOutput);
  }

  public void pivotTo(double setpoint) {
    pivotTo(setpoint, false);
  }

  public void pivotTo(double setpoint, boolean aggressive) {
    m_aggressiveComponent = aggressive ? Math.signum(setpoint) * EndEffectorConstants.kAgressiveComponent : 0;
    targetRotation = setpoint; //TODO: clamp setpoint
  }

  public double getSetpoint() {
    return targetRotation;
  }

  // commented out because this code does not make sense
  // pivot motor should be controlled with a pid in fastperiodic
  // maybe these should be m_coralMotor or m_algaeMotor?
  public void intakeAlgae(){
    effectorOutput = EndEffectorConstants.kAlgaeIntakeSpeed;
  }

  public void intakeCoral(){
    if (m_endEffectorRange.getDistance().getValueAsDouble() != 0) {
      effectorOutput = EndEffectorConstants.kCoralIntakeSpeed;
    }
  }

  public void outtakeAlgae(){
    effectorOutput = EndEffectorConstants.kAlgaeOuttakeSpeed;
  }

  public void outtakeCoral(){
    effectorOutput = EndEffectorConstants.kCoralOuttakeSpeed;
  }

  public void stopEffector() {
    effectorOutput = 0;
  }

  public void setSpeed(double speed) {
    m_aggressiveComponent = 0;
    m_speedOverride = speed;
  }

  public double getPivotPosition() {
    final double encoderPosition = m_pivotMotor.getAbsoluteEncoder().getPosition();
    return encoderPosition >= EndEffectorConstants.kPivotWraparoundPoint ? 0 : encoderPosition;
  }

  /**
   * Checks if currently holding
   * @return True if either a coral or algae is currently being held
   */
  public boolean isHolding() {
    return m_endEffectorRange.getDistance().getValueAsDouble() <= EndEffectorConstants.kSensorDistanceThreshold;
  }

  public boolean atSetpoint() {
    return m_PIDController.atSetpoint();
  }
}
