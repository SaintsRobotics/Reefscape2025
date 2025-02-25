// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  private final SparkFlex m_pivotMotor;
  private final SparkFlex m_effectorMotor; //TODO: confirm with mech that these are actually two different motors
  private final CANrange m_endEffectorRange = new CANrange(EndEffectorConstants.kEndEffectorCANrangePort);

  private final PIDController m_PIDController = new PIDController(EndEffectorConstants.kPEndEffector, 0, 0, Constants.kFastPeriodicPeriod);

  private double targetRotation = 0;

  private final DoubleSupplier m_elevatorHeightSupplier;

  // Pivoting controls: A is L1, B is L2 & L3, Y is L4 or use right joystick
  // Intake/Outtake controls: Right Bumper: Intake Algae, Left Bumper: Outtake Algae
  //                          Right Trigger: Intake Coral, Left Trigger Outtake Coral

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem(DoubleSupplier elevatorHeightSupplier) {
    m_pivotMotor = new SparkFlex(EndEffectorConstants.kPivotMotorPort, MotorType.kBrushless);
    m_effectorMotor = new SparkFlex(EndEffectorConstants.kEffectorMotorPort, MotorType.kBrushless);

    m_elevatorHeightSupplier = elevatorHeightSupplier;
    m_PIDController.setTolerance(EndEffectorConstants.kPivotTolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void fastPeriodic(){
    double output = m_PIDController.calculate(m_pivotMotor.getEncoder().getPosition(), targetRotation);
    output = MathUtil.clamp(output, -EndEffectorConstants.kPivotMaxSpeed, EndEffectorConstants.kPivotMaxSpeed);
    m_pivotMotor.set(output);
  }

  public void pivotTo(double setpoint) {
    final double elevatorHeight = m_elevatorHeightSupplier.getAsDouble();
    final Pair<Double, Double> limits = EndEffectorConstants.kSafePivotPositions.floorEntry(elevatorHeight).getValue();
    targetRotation = MathUtil.clamp(setpoint, limits.getFirst(), limits.getSecond());
  }

  /**
   * Checks is pivot is within elevator limits
   * @return true if pivot is within limits
   */
  public boolean pivotWithinLimits() {
    final double elevatorHeight = m_elevatorHeightSupplier.getAsDouble();
    final Pair<Double, Double> limits = EndEffectorConstants.kSafePivotPositions.floorEntry(elevatorHeight).getValue();
    return targetRotation >= limits.getFirst() && targetRotation <= limits.getSecond(); 
  }

  /**
   * Ensures pivot is at a safe state for elevator
   * Should be called whenever elevator height is changed
   */
  public void ensureSafeState() {
    pivotTo(targetRotation); // will re-evaluate clamp
  }

  // commented out because this code does not make sense
  // pivot motor should be controlled with a pid in fastperiodic
  // maybe these should be m_coralMotor or m_algaeMotor?
  public void intakeAlgae(){
    m_effectorMotor.set(EndEffectorConstants.kAlgaeIntakeSpeed);
  }

  public void intakeCoral(){
    if (m_endEffectorRange.getDistance().getValueAsDouble() != 0) m_effectorMotor.set(EndEffectorConstants.kCoralIntakeSpeed);
  }

  public void outtakeAlgae(){
    m_effectorMotor.set(EndEffectorConstants.kAlgaeOuttakeSpeed);
  }

  public void outtakeCoral(){
    m_effectorMotor.set(EndEffectorConstants.kCoralOuttakeSpeed);
  }
}
