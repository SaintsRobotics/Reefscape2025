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

import edu.wpi.first.math.MathUtil;
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

  private final Interlocks m_interlocks;

  private boolean m_overrideSetpoint;
  private double m_speedOverride;

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
    m_overrideSetpoint = false;
  }

  @Override
  public void periodic() {
    m_interlocks.setPivotPosition(getPivotPosition());

    SmartDashboard.putBoolean("Is Holding", isHolding());

    // This method will be called once per scheduler run
  }

  public void fastPeriodic(){
    double output = m_PIDController.calculate(getPivotPosition(), targetRotation);
    output = MathUtil.clamp(output, -EndEffectorConstants.kPivotMaxSpeed, EndEffectorConstants.kPivotMaxSpeed);
    m_pivotMotor.set(m_interlocks.clampPivotMotorSet(m_overrideSetpoint ? m_speedOverride : output));
    m_effectorMotor.set(effectorOutput);
  }

  public void pivotTo(double setpoint) {
    targetRotation = m_interlocks.clampPivotMotorSetpoint(setpoint);
    m_overrideSetpoint = false;
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

  public void setSpeed(double speed) {
    m_speedOverride = speed;
    m_overrideSetpoint = true;
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
