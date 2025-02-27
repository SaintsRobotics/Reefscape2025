// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map.Entry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PlaceGrabAlgaeCommand;
import frc.robot.commands.PlaceGrabCoralCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.utils.Interlocks;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final Interlocks m_interlocks = new Interlocks();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem(m_interlocks);
  private final EndEffectorSubsystem m_endEffector = new EndEffectorSubsystem(m_interlocks);

  private final XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(IOConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(
                    -m_driverController.getLeftY(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - m_driverController
                        .getLeftTriggerAxis()
                        * IOConstants.kSlowModeScalar)
                    * 0.8,
                MathUtil.applyDeadband(
                    -m_driverController.getLeftX(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - m_driverController
                        .getLeftTriggerAxis()
                        * IOConstants.kSlowModeScalar)
                    * 0.8,
                MathUtil.applyDeadband(
                    m_driverController.getRightX(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxAngularSpeedRadiansPerSecond
                    * (1 - m_driverController
                        .getLeftTriggerAxis()
                        * IOConstants.kSlowModeScalar)
                    * -1,
                !m_driverController.getRightBumperButton()),
                    m_robotDrive));

    m_elevator.setDefaultCommand(new RunCommand(() -> m_elevator.setHeight(m_elevator.getCurrentHeight()), m_elevator));
}

  /**
   * Use this method to define your button->command mappings.
   * 
   * Driver Controls:
   *    left axis X/Y:                  movement speed
   *    right axis X:                   turn speed
   *    left trigger:                   slow mode
   *    right bumper:                   robot relative
   *    start:                          zero heading
   *    back:                           reset gyro
   * 
   * Operator Controls:
   *    left axis Y (B unpressed):      semi-automatic elevator speed
   *    left axis Y (B pressed):        manual elevator speed
   *    right axis Y:                   manual pivot speed
   *    A (right bumper unpressed):     intake algae
   *    A (right bumper pressed):       outtake algae
   *    X (right bumper unpressed):     intake coral
   *    X (right bumper pressed):       outtake coral
   *    Dpad up:                        L1 elevator position
   *    Dpad right:                     L2 elevator position
   *    Dpad down:                      L3 elevator position
   *    Dpad left:                      L4 elevator position
   *    right trigger:                  place/grab algae
   *    left trigger:                   place/grab coral
   */
  private void configureBindings() {
    
    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    new JoystickButton(m_driverController, Button.kBack.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d()), m_robotDrive));

    new JoystickButton(m_operatorController, Button.kRightBumper.value).negate()
            .and(m_operatorController::getAButton)
            .whileTrue(new RunCommand(m_endEffector::intakeAlgae, m_endEffector));

    new JoystickButton(m_operatorController, Button.kRightBumper.value)
            .and(m_operatorController::getAButton)
            .whileTrue(new RunCommand(m_endEffector::outtakeAlgae, m_endEffector));

    new JoystickButton(m_operatorController, Button.kRightBumper.value).negate()
            .and(m_operatorController::getXButton)
            .whileTrue(new RunCommand(m_endEffector::intakeCoral, m_endEffector));

    new JoystickButton(m_operatorController, Button.kRightBumper.value)
            .and(m_operatorController::getXButton)
            .whileTrue(new RunCommand(m_endEffector::outtakeCoral, m_endEffector));

    // full manual elevator
    new Trigger(m_operatorController::getBButton)
            .and(() -> MathUtil.applyDeadband(m_operatorController.getLeftY(), IOConstants.kControllerDeadband) != 0)
            .whileTrue(new RunCommand(() -> {
                m_elevator.setSpeed(-m_operatorController.getLeftY() * IOConstants.kElevatorAxisScalar); // no need to apply deadband here because of trigger
            }, m_elevator));

    // semi manual elevator
    new Trigger(m_operatorController::getBButton).negate()
            .and(() -> MathUtil.applyDeadband(m_operatorController.getLeftY(), IOConstants.kControllerDeadband) != 0)
            .whileTrue(new RunCommand(() -> {
                // because we cant do position prediction here, we need to use more restrictive
                // pivot adjustments
                // always clamp using current, and also clamp to next
                final double speed = -m_operatorController.getLeftY() * IOConstants.kElevatorAxisScalar; // no need to apply deadband here because of trigger

                double pivotSetpoint = m_endEffector.getSetpoint();
                final double currentPosition = m_elevator.getCurrentHeight();

                final Entry<Double, Pair<Double, Double>> currentLimit = EndEffectorConstants.kSafePivotPositions
                        .floorEntry(currentPosition);
                final Entry<Double, Pair<Double, Double>> higherLimit = EndEffectorConstants.kSafePivotPositions
                        .higherEntry(currentPosition);
                final Entry<Double, Pair<Double, Double>> lowerLimit = EndEffectorConstants.kSafePivotPositions
                        .lowerEntry(currentLimit.getKey());

                // clamp to current
                pivotSetpoint = MathUtil.clamp(pivotSetpoint, currentLimit.getValue().getFirst(),
                        currentLimit.getValue().getSecond());

                // check direction
                if (speed > 0) { // going up
                    pivotSetpoint = MathUtil.clamp(pivotSetpoint, higherLimit.getValue().getFirst(),
                            higherLimit.getValue().getSecond());
                }
                else { // going down
                    pivotSetpoint = MathUtil.clamp(pivotSetpoint, lowerLimit.getValue().getFirst(),
                        lowerLimit.getValue().getSecond());
                }

                m_endEffector.pivotTo(pivotSetpoint);
                m_elevator.setSpeed(speed);
            }, m_elevator, m_endEffector));

    // pivot
    new Trigger(() -> MathUtil.applyDeadband(m_operatorController.getRightY(), IOConstants.kControllerDeadband) != 0)
            .whileTrue(new RunCommand(() -> {
                m_endEffector.setSpeed(-m_operatorController.getRightY() * IOConstants.kPivotAxisScalar); // no need to apply deadband here because of trigger
            }, m_endEffector));

    // auto intake/outake
    new Trigger(() -> m_operatorController.getRightTriggerAxis() > IOConstants.kControllerDeadband).whileTrue(new PlaceGrabAlgaeCommand(m_endEffector));
    new Trigger(() -> m_operatorController.getLeftTriggerAxis() > IOConstants.kControllerDeadband).whileTrue(new PlaceGrabCoralCommand(m_endEffector));

    new POVButton(m_operatorController, IOConstants.kDPadUp) // Up - L1
        .onTrue(new ElevatorCommand(ElevatorConstants.kL1Height, m_elevator, m_endEffector));
    new POVButton(m_operatorController, IOConstants.kDPadRight) // Right - L2
        .onTrue(new ElevatorCommand(ElevatorConstants.kL1Height, m_elevator, m_endEffector));
    new POVButton(m_operatorController, IOConstants.kDPadDown) // Down - L3
        .onTrue(new ElevatorCommand(ElevatorConstants.kL1Height, m_elevator, m_endEffector));
    new POVButton(m_operatorController, IOConstants.kDPadLeft) // Left - L4
        .onTrue(new ElevatorCommand(ElevatorConstants.kL1Height, m_elevator, m_endEffector));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  /**
   * This periodic loop runs every 10ms (100Hz)
   * 
   * <p>
   * Should be used for any code that needs to be run more frequently than the
   * default 20ms loop (50Hz) such as PID Controllers.
   * </p>
   */
  public void fastPeriodic() {
    m_robotDrive.fastPeriodic();
    m_elevator.fastPeriodic(); // Temporarily commented out to merge
    m_endEffector.fastPeriodic(); // Temporarily commented out
  }
}
