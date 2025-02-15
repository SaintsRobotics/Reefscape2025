// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.pathplanner.lib.events.CancelCommandEvent;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  private final SparkFlex m_pivotMotor;
  private final SparkFlex m_coralMotor;
  private final SparkFlex m_algaeMotor;
  private final CANrange m_EndEffectorRange = new CANrange(0);

  private final PIDController m_PIDController = new PIDController(EndEffectorConstants.kPEndEffector, 0, 0);

  private double targetRotation = 0;

  // Pivoting controls: A is L1, B is L2 & L3, Y is L4 or use right joystick
  // Intake/Outtake controls: Right Bumper: Intake Algae, Left Bumper: Outtake Algae
  //                          Right Trigger: Intake Coral, Left Trigger Outtake Coral

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
    m_pivotMotor = new SparkFlex(EndEffectorConstants.kPivotMotorPort, MotorType.kBrushless);
    m_coralMotor = new SparkFlex(EndEffectorConstants.kCoralMotorPort, MotorType.kBrushless);
    m_algaeMotor = new SparkFlex(EndEffectorConstants.kAlgaeMotorPort, MotorType.kBrushless);

    
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
    targetRotation = setpoint;
  }

  public void intakeAlgae(){
    m_pivotMotor.set(EndEffectorConstants.kAlgaeIntakeSpeed);
  }

  public void intakeCoral(){
    if (m_EndEffectorRange.getDistance().getValueAsDouble() != 0) m_pivotMotor.set(EndEffectorConstants.kCoralIntakeSpeed);

  }

  public void outtakeAlgae(){
    m_pivotMotor.set(EndEffectorConstants.kAlgaeOuttakeSpeed);
  }

  public void outtakeCoral(){
    m_pivotMotor.set(EndEffectorConstants.kCoralOuttakeSpeed);
  }
}
