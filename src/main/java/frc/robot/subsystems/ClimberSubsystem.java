// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex m_windingMotor;
  private final Servo m_lockingServo;

  private final PIDController m_PIDController = new PIDController(ClimberConstants.kPWindingMotor, 0, 0, Constants.kFastPeriodicPeriod);

  private double m_windingSetpoint = 0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    SparkFlexConfig motorConfig = new SparkFlexConfig();
    motorConfig.encoder.positionConversionFactor(ClimberConstants.kWindingMotorGearing);

    m_windingMotor = new SparkFlex(ClimberConstants.kWindingMotorPort, MotorType.kBrushless);
    m_windingMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_lockingServo = new Servo(ClimberConstants.kLockingServoPWMPort);
  } 

  public void fastPeriodic() {
    double output = m_PIDController.calculate(m_windingMotor.getEncoder().getPosition(),
    m_windingSetpoint) + ClimberConstants.kWindingMotorFeedForward;

    output = MathUtil.clamp(output, -ClimberConstants.kWindingMotorMaxSpeed, ClimberConstants.kWindingMotorMaxSpeed);
    m_windingMotor.set(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLockPosition(int lockPosition) {
    m_lockingServo.setAngle(lockPosition);
  }

  public void setWindingSetpoint(double setpoint) {
    m_windingSetpoint = setpoint;
  }

  public boolean windingAtSetpoint() {
    return m_PIDController.atSetpoint();
  }
}
