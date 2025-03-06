// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveToReef;
// import frc.robot.commands.ElevatorCommand;
// import frc.robot.commands.ElevatorSemiAutomaticDriveCommand;
import frc.robot.commands.PivotCommand;
// import frc.robot.commands.PlaceGrabAlgaeCommand;
// import frc.robot.commands.PlaceGrabCoralCommand;
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

    // Defaults to keep the elevator in its current state, unmoving
    m_elevator.setDefaultCommand(new StartEndCommand(() -> {
            m_elevator.setHeight(m_elevator.getCurrentHeight());
    }, () -> {
    }, m_elevator));

    // default to keep the end effector in its current state, unmoving
    m_endEffector.setDefaultCommand(new StartEndCommand(() -> {
            m_endEffector.pivotTo(m_endEffector.getPivotPosition());
    }, () -> {
    }, m_endEffector));
}

public void initSubsystems() {
    // cancel commands
    new InstantCommand(() -> {
    }, m_elevator, m_endEffector).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();

    // Assumes the elevator starts at 0 on 
    m_elevator.zeroPosition();
    m_elevator.setHeight(m_elevator.getCurrentHeight());
    m_endEffector.pivotTo(m_endEffector.getPivotPosition());
}

  /**
   * Use this method to define your button->command mappings.
   * 
   * Driver Controls:
   *    left axis X/Y:                  robot translation
   *    right axis X:                   robot rotation
   *    left trigger:                   slow mode
   *    right bumper:                   robot relative
   *    start:                          zero heading
   *    back:                           reset gyro
   *    A (left bumper pressed):        auto align to reef
   * 
   * Operator Controls:
   *    left axis Y (B unpressed):      semi-automatic elevator speed
   *    left axis Y (B pressed):        manual elevator speed
   *    right axis Y:                   manual pivot speed
   *    A (right bumper unpressed):     intake algae
   *    A (right bumper pressed):       outtake algae
   *    X (right bumper unpressed):     intake coral
   *    X (right bumper pressed):       outtake coral
   *    start (left bumper pressed):    increment elevator (see 1)
   *    back (left bumper presssed):    reset elevator (see 2)
   *    Dpad up:                        L1 elevator position
   *    Dpad right:                     L2 elevator position
   *    Dpad down:                      L3 elevator position
   *    Dpad left:                      L4 elevator position
   *    right trigger:                  grab algae
   *    Y button:                      place algae
   *    left trigger:                   place/grab coral
   * 
   *    1: Increments both the elevator offset and setpoint.
   *        Does not cause any movement. Used to move elevator
   *        below zero when not calibrated. Effect does not
   *        stack
   *    2: Resets position, offset and setpoint
   *        Does not cause any movement. Used to reset elevator
   *        position when distance sensor fails
   */
  private void configureBindings() {
    
    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    new JoystickButton(m_driverController, Button.kBack.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d()), m_robotDrive));

    // auto align to reef
    new JoystickButton(m_driverController, Button.kA.value)
      .whileTrue(new DriveToReef(m_robotDrive, () -> m_driverController.getLeftBumperButton()));

    // intake TODO: yes this overlaps with slow mode
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5)
        .onTrue(new InstantCommand(() -> {
          if (m_interlocks.getCoralMode()) {
            m_endEffector.intakeCoral();
          } else {
            m_endEffector.intakeAlgae();
          }
        }));

    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5)
        .onTrue(new InstantCommand(() -> {
          if (m_interlocks.getCoralMode()) {
            m_endEffector.outtakeCoral();
          } else {
            m_endEffector.outtakeAlgae();
          }
        }));

  //     // .onTrue(new InstantCommand(() -> m_endEffector.intakeCoral(), m_endEffector));
  //       .onTrue(new InstantCommand(() -> {
  //         if (m_interlocks.getCoralMode()) {
  //             m_endEffector.intakeCoral();
  // }
  // else {
  //             m_endEffector.intakeAlgae();
  // } }));



    //     new InstantCommand(() -> { return m_interlocks.getCoralMode() ? m_endEffector.intakeCoral() : m_endEffector.intakeAlgae(); });

    // elevator from joystick - manual (only interlocks remaining)
    new Trigger(() -> MathUtil.applyDeadband(m_operatorController.getLeftY(), IOConstants.kControllerDeadband) != 0)
        .whileTrue(new RunCommand(() -> {
            // no need to apply deadband here because of trigger
            m_elevator.setSpeed(-m_operatorController.getLeftY() * IOConstants.kElevatorAxisScalar);
        }, m_elevator));

    // pivot from joystick - manual (only interlocks remaining)
    new Trigger(() -> MathUtil.applyDeadband(m_operatorController.getRightY(), IOConstants.kControllerDeadband) != 0)
        .whileTrue(new RunCommand(() -> {
            // no need to apply deadband here because of trigger
            m_endEffector.setSpeed(-m_operatorController.getRightY() * IOConstants.kPivotAxisScalar);
        }, m_endEffector));

    // algae mode
    new JoystickButton(m_operatorController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_interlocks.setCoralMode(false)))
        .onTrue(new PivotCommand(m_endEffector, EndEffectorConstants.kAlgaeAngle));

    // coral mode
    new JoystickButton(m_operatorController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_interlocks.setCoralMode(true)));

    new POVButton(m_operatorController, IOConstants.kDPadUp) // Up - L1
        .onTrue(m_interlocks.getCoralMode()
            ? getCoralCommand(1, m_elevator, m_endEffector)
            : getAlgaeCommand(1, EndEffectorConstants.kAlgaeAngle, m_elevator, m_endEffector));
    new POVButton(m_operatorController, IOConstants.kDPadRight) // Right - L2
        .onTrue(m_interlocks.getCoralMode()
            ? getCoralCommand(2, m_elevator, m_endEffector)
            : getAlgaeCommand(2, EndEffectorConstants.kAlgaeAngle, m_elevator, m_endEffector));
    new POVButton(m_operatorController, IOConstants.kDPadDown) // Down - L3
        .onTrue(m_interlocks.getCoralMode()
            ? getCoralCommand(3, m_elevator, m_endEffector)
            : getAlgaeCommand(3, EndEffectorConstants.kAlgaeAngle, m_elevator, m_endEffector));
    new POVButton(m_operatorController, IOConstants.kDPadLeft) // Left - L4
        .onTrue(m_interlocks.getCoralMode()
            ? getCoralCommand(4, m_elevator, m_endEffector)
            : getAlgaeCommand(4, EndEffectorConstants.kBargeAngle, m_elevator, m_endEffector));
  }

  public SequentialCommandGroup getCoralCommand(int level, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {

    /*
     * possibilites
     * 
     * literally map safe pivot angles to go from things like l1 to l2, l3, l4
     * l1 -> l3 find safe angle to move elevator at
     * l1 -> l4 find safe angle to move elevator at
     * etc
     * 
     * if setpoint is not at any level, move pivot all the way out and elevator all the way down
     * literally set both setpoints and use discrete interlocks ig to make sure it doesnt hit
     */

     // new InstantCommand(() -> {
    //   elevatorSubsystem.setHeight(ElevatorConstants.kCoralL1Height);
    //   endEffectorSubsystem.pivotTo(desiredAngle);
    // }, elevatorSubsystem, endEffectorSubsystem)

    // TODO: add safe movement angle or figure out if it needs to change based on where elevator is starting
    double safeMovementAngle = EndEffectorConstants.kPivotSafeAngle;
    
    switch (level) {
      case 1:
        return new SequentialCommandGroup(
          new PivotCommand(endEffectorSubsystem, safeMovementAngle),
          new InstantCommand(() -> elevatorSubsystem.setHeight(ElevatorConstants.kCoralL1Height), elevatorSubsystem),
          new PivotCommand(endEffectorSubsystem, EndEffectorConstants.kL1Pivot));

      case 2:
        return new SequentialCommandGroup(
          new PivotCommand(endEffectorSubsystem, safeMovementAngle),
          new InstantCommand(() -> elevatorSubsystem.setHeight(ElevatorConstants.kCoralL1Height), elevatorSubsystem),
          new PivotCommand(endEffectorSubsystem, EndEffectorConstants.kL23Pivot));

      case 3:
        return new SequentialCommandGroup(
          new PivotCommand(endEffectorSubsystem, safeMovementAngle),
          new InstantCommand(() -> elevatorSubsystem.setHeight(ElevatorConstants.kCoralL1Height), elevatorSubsystem),
          new PivotCommand(endEffectorSubsystem, EndEffectorConstants.kL23Pivot));

      case 4:
      return new SequentialCommandGroup(
        new PivotCommand(endEffectorSubsystem, safeMovementAngle),
        new InstantCommand(() -> elevatorSubsystem.setHeight(ElevatorConstants.kCoralL1Height), elevatorSubsystem),
        new PivotCommand(endEffectorSubsystem, EndEffectorConstants.kL4Pivot));   

      default:
        return new SequentialCommandGroup();
    }
  }

  public SequentialCommandGroup getAlgaeCommand(double height, double algaeAngle, ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    return new SequentialCommandGroup(
      new PivotCommand(endEffectorSubsystem, algaeAngle),
      new InstantCommand(() -> elevatorSubsystem.setHeight(height), elevatorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
    // return new InstantCommand(() -> {m_robotDrive.resetOdometry(new Pose2d(new Translation2d(5.81, 3.86), Rotation2d.fromDegrees(180)));}, m_robotDrive);
    // return new DriveToPose(m_robotDrive, new Pose2d(new Translation2d(5.81, 3.86), Rotation2d.fromDegrees(180)));
    // return new DriveToReef(m_robotDrive);
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
    m_elevator.fastPeriodic();
    m_endEffector.fastPeriodic();
  }
}
