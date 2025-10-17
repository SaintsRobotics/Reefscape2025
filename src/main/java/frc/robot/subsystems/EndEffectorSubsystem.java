// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.utils.Interlocks;

public class EndEffectorSubsystem extends SubsystemBase {
  private final SparkFlex m_pivotMotor;
  private final SparkFlex m_effectorMotor;
  private final CANrange m_endEffectorRange = new CANrange(EndEffectorConstants.kEndEffectorCANrangePort);

  private final PIDController m_PIDController = new PIDController(EndEffectorConstants.kPEndEffector, 0, 0,
      Constants.kFastPeriodicPeriod);

  private double targetRotation = 0;
  private double effectorOutput = 0;

  private double m_pivotOutput;

  private final Interlocks m_interlocks;

  private double m_speedOverride;

  private double m_aggressiveComponent;

  public enum IntakeState {
    IntakeCoral,
    OuttakeCoral,
    IntakeAlgae,
    OuttakeAlgae,
    ReverseCoral,
    ForceCoral,
    Idle
  }

  public IntakeState m_intakeState = IntakeState.Idle;

  // simulation objects
  private final SingleJointedArmSim m_armSim;
  private final MechanismLigament2d m_armMech;
  private final DCMotor m_pivotDCMotor;
  private final SparkFlexSim m_pivotMotorSim;

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem(Interlocks interlocks) {
    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.absoluteEncoder.positionConversionFactor(Math.PI * 2); // convert to radians

    SparkFlexConfig effectorConfig = new SparkFlexConfig();
    effectorConfig.idleMode(IdleMode.kBrake);

    m_pivotMotor = new SparkFlex(EndEffectorConstants.kPivotMotorPort, MotorType.kBrushless);
    m_effectorMotor = new SparkFlex(EndEffectorConstants.kEffectorMotorPort, MotorType.kBrushless);

    // TODO: set to reset and persist after testing
    m_pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_PIDController.setTolerance(EndEffectorConstants.kPivotTolerance);

    m_interlocks = interlocks;
    m_aggressiveComponent = 0;

    // initialize simulation objects
    m_pivotDCMotor = DCMotor.getNeoVortex(1);
    m_armSim = new SingleJointedArmSim(
      m_pivotDCMotor, 
      1, 
      1, 
      0.4, 
      0.02, 
      Math.PI *  2, 
      false, 
      Math.PI / 2);
    m_pivotMotorSim = new SparkFlexSim(m_pivotMotor, m_pivotDCMotor);

    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("end effector", 1.5, 1.5);
    m_armMech = root.append(new MechanismLigament2d("pivot", 1, 0));

    SmartDashboard.putData("end effector sim", mech);
  }

  @Override
  public void periodic() {
    m_interlocks.setPivotPosition(getPivotPosition());

    /*
     * The order of callbacks is as follows:
     * The timed robot periodic will run
     * Then the command command scheduler will run
     * Then all periodics will run
     * Then all commands will run
     * Then the fast periodics will run
     * Then the fast periodics will run again
     * 
     * This means that we will set overrideSpeed to 0 in each periodic
     * Then a command might cause this to become non zero
     * In that case, the two fast periodics will use the speed override instead of
     * the setpoint
     */

    m_speedOverride = 0;

    SmartDashboard.putBoolean("Is Holding", isHolding());
    SmartDashboard.putNumber("Pivot Angle 2", getPivotPosition());
    SmartDashboard.putNumber("raw rotations", m_pivotMotor.getAbsoluteEncoder().getPosition() / Math.PI / 2.0);
    SmartDashboard.putNumber("Pivot Output", m_pivotOutput);
    SmartDashboard.putNumber("Pivot clamped output", m_pivotMotor.get());
    SmartDashboard.putNumber("Pivot Setpoint", m_PIDController.getSetpoint());
    SmartDashboard.putString("Intake state", m_intakeState.toString());
  }

  public void fastPeriodic() {
    // m_pivotOutput = -m_PIDController.calculate(getPivotPosition(), targetRotation + m_aggressiveComponent);
    m_pivotOutput = m_PIDController.calculate(getPivotPosition(), targetRotation + m_aggressiveComponent);
    m_pivotOutput = m_speedOverride != 0 ? m_speedOverride : m_pivotOutput;

    switch (m_intakeState) {
      case IntakeCoral:
        // We are doing this check here in fast periodic so we react to intookened coral faster
        if (!isHolding()) {
          effectorOutput = EndEffectorConstants.kCoralIntakeSpeed;
        } else {
          m_intakeState = IntakeState.Idle;
          effectorOutput = 0;
        }
        break;
      case IntakeAlgae:
        effectorOutput = EndEffectorConstants.kAlgaeIntakeSpeed;
        break;
      case OuttakeAlgae:
        effectorOutput = EndEffectorConstants.kAlgaeOuttakeSpeed;
        break;
      case OuttakeCoral:
        effectorOutput = EndEffectorConstants.kCoralOuttakeSpeed;
        break;
      case ReverseCoral:
        effectorOutput = EndEffectorConstants.kCoralReverseSpeed;
        break;
      case ForceCoral:
        effectorOutput = EndEffectorConstants.kCoralIntakeSpeed;
        break;
      case Idle:
        effectorOutput = 0;
        break;
    }

    m_pivotMotor.set(m_interlocks.clampPivotMotorSet(m_pivotOutput));
    m_effectorMotor.set(effectorOutput);
  }

  public void pivotTo(double setpoint) {
    pivotTo(setpoint, false);
  }

  public void pivotTo(double setpoint, boolean aggressive) {
    m_aggressiveComponent = aggressive ? Math.signum(setpoint) * EndEffectorConstants.kAgressiveComponent : 0;
    targetRotation = setpoint; // TODO: clamp setpoint
    m_PIDController.setSetpoint(targetRotation + m_aggressiveComponent);
  }

  public double getSetpoint() {
    return targetRotation;
  }

  public void setIntakeState(IntakeState intakeState) {
    m_intakeState = intakeState;
  }

  public IntakeState getIntakeState() {
    return m_intakeState;
  }

  public void intakeAlgae() {
    m_intakeState = IntakeState.IntakeAlgae;
  }

  public void intakeCoral() {
    if (m_endEffectorRange.getDistance().getValueAsDouble() != 0) {
      m_intakeState = IntakeState.IntakeCoral;
    }
  }

  public void outtakeAlgae() {
    m_intakeState = IntakeState.OuttakeAlgae;
  }

  public void outtakeCoral() {
    m_intakeState = IntakeState.OuttakeCoral;
  }

  public void reverseCoral() {
    m_intakeState = IntakeState.ReverseCoral;
  }

  public void forceCoral() {
    m_intakeState = IntakeState.ForceCoral;
  }

  public void stopEffector() {
    m_intakeState = IntakeState.Idle;
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
   * 
   * @return True if either a coral or algae is currently being held
   */
  public boolean isHolding() {
    return m_endEffectorRange.getDistance().getValueAsDouble() <= EndEffectorConstants.kSensorDistanceThreshold;
  }

  public boolean atSetpoint() {
    return m_PIDController.atSetpoint();
  }

  public void simulationPeriodic() {
    // update simulated pivot input
    m_armSim.setInput(m_pivotMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // update arm simulation
    m_armSim.update(0.02);

    // iterate simulated SparkFlex and update simulated encoder
    m_pivotMotorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(m_armSim.getVelocityRadPerSec()), RobotController.getBatteryVoltage(), 0.02);
    m_pivotMotorSim.getAbsoluteEncoderSim().setPosition(m_armSim.getAngleRads());

    // update battery
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
    
    // update mechanism2d
    m_armMech.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
  }
}
