// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutonConstants;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.AutonCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveToReef;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorSemiAutomaticDriveCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.PlaceGrabAlgaeCommand;
import frc.robot.commands.PlaceGrabCoralCommand;
import frc.robot.commands.SmartPivotCommand;
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

    private final SendableChooser<Command> m_autoChooser;
  private boolean m_coralMode = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putBoolean("coral mode", m_coralMode);

    // Configure the trigger bindings
    configureBindings();

        AutoBuilder.configure(m_robotDrive::getPose, (pose) -> m_robotDrive.resetOdometry(pose), () -> m_robotDrive.getRobotRelativeSpeeds(),
                (speeds) -> m_robotDrive.autonDrive(speeds),
                new PPHolonomicDriveController(AutonConstants.kTranslationConstants, AutonConstants.kRotationConstants),
                AutonConstants.kBotConfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, m_robotDrive);


        AutonCommands.registerAutonCommands();

        m_autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(m_autoChooser);

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(
                    -m_driverController.getLeftY(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - (m_driverController
                        .getRightBumperButton() ? IOConstants.kSlowModeScalar : 0))
                    * 0.8,
                MathUtil.applyDeadband(
                    -m_driverController.getLeftX(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxSpeedMetersPerSecond
                    * (1 - (m_driverController
                        .getRightBumperButton() ? 1 : 0)
                        * IOConstants.kSlowModeScalar)
                    * 0.8,
                MathUtil.applyDeadband(
                    m_driverController.getRightX(),
                    IOConstants.kControllerDeadband)
                    * DriveConstants.kMaxAngularSpeedRadiansPerSecond
                    * (1 - (m_driverController
                        .getRightBumperButton() ? 1 : 0)
                        * IOConstants.kSlowModeScalar)
                    * -1,
                true),
                    m_robotDrive));

    m_elevator.setDefaultCommand(new StartEndCommand(() -> {
        m_elevator.setHeight(m_elevator.getCurrentHeight());
    }, () -> {
    }, m_elevator));
    m_endEffector.setDefaultCommand(new StartEndCommand(() -> {
        m_endEffector.pivotTo(m_endEffector.getPivotPosition());
    }, () -> {
    }, m_endEffector));
}

public void initSubsystems() {
    // cancel commands
    new InstantCommand(() -> {
    }, m_elevator, m_endEffector).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();

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
   *    Y button:                       place algae
   *    left trigger:                   place/grab coral
   *    right stick press               smart pivot
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

    // new JoystickButton(m_operatorController, Button.kRightBumper.value).negate()
    //                 .and(m_operatorController::getAButton)
    //                 .whileTrue(new RunCommand(m_endEffector::intakeAlgae, m_endEffector).alongWith(
    //                                 new InstantCommand(() -> m_interlocks.setAlgeaHolding(true))))
    //                                 .onFalse(new InstantCommand(m_endEffector::stopEffector, m_endEffector));

    // new JoystickButton(m_operatorController, Button.kRightBumper.value)
    //                 .and(m_operatorController::getAButton)
    //                 .whileTrue(new RunCommand(m_endEffector::outtakeAlgae, m_endEffector).alongWith(
    //                                 new InstantCommand(() -> m_interlocks.setAlgeaHolding(false))))
    //                                 .onFalse(new InstantCommand(m_endEffector::stopEffector, m_endEffector));

    // new JoystickButton(m_operatorController, Button.kRightStick.value)
    //     .whileTrue(new SmartPivotCommand(m_endEffector, m_elevator::getCurrentHeight));

    // new JoystickButton(m_operatorController, Button.kRightBumper.value).negate()
    //         .and(m_operatorController::getXButton)
    //         .whileTrue(new RunCommand(m_endEffector::intakeCoral, m_endEffector))
    //         .onFalse(new InstantCommand(m_endEffector::stopEffector, m_endEffector));

    // new JoystickButton(m_operatorController, Button.kRightBumper.value)
    //         .and(m_operatorController::getXButton)
    //         .whileTrue(new RunCommand(m_endEffector::outtakeCoral, m_endEffector))
    //         .onFalse(new InstantCommand(m_endEffector::stopEffector, m_endEffector));

    new JoystickButton(m_operatorController, Button.kA.value)
      .onTrue(new InstantCommand(m_endEffector::intakeCoral, m_endEffector))
      .onFalse(new InstantCommand(m_endEffector::stopEffector, m_endEffector));

    new JoystickButton(m_operatorController, Button.kX.value)
      .onTrue(new RunCommand(m_endEffector::outtakeCoral, m_endEffector))
      .onFalse(new InstantCommand(m_endEffector::stopEffector, m_endEffector));

    new JoystickButton(m_operatorController, Button.kStart.value)
            .onTrue(new InstantCommand(() -> m_elevator.zeroPosition(5), m_elevator));

    new JoystickButton(m_operatorController, Button.kBack.value)
            .onTrue(new InstantCommand(() -> m_elevator.zeroPosition(), m_elevator));

    // full manual elevator
    new JoystickButton(m_operatorController, Button.kB.value)
            .and(() -> MathUtil.applyDeadband(m_operatorController.getLeftY(), IOConstants.kControllerDeadband) != 0)
            .whileTrue(new RunCommand(() -> {
                m_elevator.setSpeed(-m_operatorController.getLeftY() * IOConstants.kElevatorAxisScalar); // no need to apply deadband here because of trigger
            }, m_elevator));

    // semi manual elevator
    new JoystickButton(m_operatorController, Button.kB.value).negate()
            .and(() -> MathUtil.applyDeadband(-m_operatorController.getLeftY(),
                    IOConstants.kControllerDeadband) != 0)
            .whileTrue(new ElevatorSemiAutomaticDriveCommand(
                    () -> -m_operatorController.getLeftY(), () -> {
                        if (m_operatorController.getLeftY() > 0) {
                            return m_elevator.getHeightSetpoint() <= m_elevator.getCurrentHeight()
                                    + ElevatorConstants.kBoundaryHintThreshold;
                        }
                        return m_elevator.getHeightSetpoint() >= m_elevator.getCurrentHeight()
                                    - ElevatorConstants.kBoundaryHintThreshold;
                    }, m_endEffector, m_elevator));

    // pivot
    new Trigger(() -> MathUtil.applyDeadband(m_operatorController.getRightY(), IOConstants.kControllerDeadband) != 0)
            .whileTrue(new RunCommand(() -> {
                m_endEffector.setSpeed(-m_operatorController.getRightY() * IOConstants.kPivotAxisScalar); // no need to apply deadband here because of trigger
            }, m_endEffector));

    // auto intake/outake
    //TODO: put actual setpoints for onFalse
    // new Trigger(() -> m_operatorController.getRightTriggerAxis() > IOConstants.kControllerDeadband)
    //         .whileTrue(new PlaceGrabAlgaeCommand(m_endEffector, false, m_interlocks))
    //         .onFalse(new PivotCommand(m_endEffector, 0));

    // new JoystickButton(m_operatorController, Button.kY.value)
    //         .whileTrue(new PlaceGrabAlgaeCommand(m_endEffector, true, m_interlocks))
    //         .onFalse(new PivotCommand(m_endEffector, 0));
    
    // new Trigger(() -> m_operatorController.getLeftTriggerAxis() > IOConstants.kControllerDeadband)
    //         .whileTrue(new ConditionalCommand(
    //             new PlaceGrabCoralCommand(m_endEffector, true),
    //             new PlaceGrabCoralCommand(m_endEffector, false),
    //             m_endEffector::isHolding));

    new JoystickButton(m_operatorController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> {
          m_coralMode = false;
          SmartDashboard.putBoolean("coral mode", m_coralMode);
        }));

    new JoystickButton(m_operatorController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> {
          m_coralMode = true;
          SmartDashboard.putBoolean("coral mode", m_coralMode);
        }));

    // Driver left trigger - intake (use m_coralMode)
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > IOConstants.kControllerDeadband)
        .whileTrue(new ConditionalCommand(
          // coral
          new PlaceGrabCoralCommand(m_endEffector, false),
          // algae
          new PlaceGrabAlgaeCommand(m_endEffector, false, m_interlocks),
          () -> m_coralMode));

    // Driver right trigger - outtake
    new Trigger(() -> m_driverController.getRightTriggerAxis() > IOConstants.kControllerDeadband)
        .whileTrue(new ConditionalCommand(
          // coral
          new PlaceGrabCoralCommand(m_endEffector, true),
          // algae
          new PlaceGrabAlgaeCommand(m_endEffector, true, m_interlocks),
          () -> m_coralMode));


    new POVButton(m_operatorController, IOConstants.kDPadUp) // Up - L1
        .onTrue(new ConditionalCommand(
            // coral
            new SequentialCommandGroup(
                new PivotCommand(m_endEffector, 1.3),
                new ElevatorCommand(ElevatorConstants.kL1Height, m_elevator, m_endEffector),
                new PivotCommand(m_endEffector, 0.05)), 
            // algae
            new SequentialCommandGroup(
                new ElevatorCommand(0, m_elevator, m_endEffector), 
                new PivotCommand(m_endEffector, 0.5)),
            () -> m_coralMode));

    new POVButton(m_operatorController, IOConstants.kDPadRight) // Right - L2
        .onTrue(new ConditionalCommand(
            // coral
            new SequentialCommandGroup(
                new PivotCommand(m_endEffector, 1.3),
                new ElevatorCommand(ElevatorConstants.kL2Height, m_elevator, m_endEffector),
                new PivotCommand(m_endEffector, 0.717)), 
            // algae
            new SequentialCommandGroup(
                new ElevatorCommand(0, m_elevator, m_endEffector), 
                new PivotCommand(m_endEffector, 0)),
            () -> m_coralMode));

    new POVButton(m_operatorController, IOConstants.kDPadDown) // Down - L3
        .onTrue(new ConditionalCommand(
            // coral
            new SequentialCommandGroup(
                new PivotCommand(m_endEffector, 1.3),
                new ElevatorCommand(ElevatorConstants.kL3Height, m_elevator, m_endEffector),
                new PivotCommand(m_endEffector, 1.035
                )), 
            // algae
            new SequentialCommandGroup(
                new ElevatorCommand(0, m_elevator, m_endEffector), 
                new PivotCommand(m_endEffector, 0.5)),
            () -> m_coralMode));

    new POVButton(m_operatorController, IOConstants.kDPadLeft) // Left - L4
        .onTrue(new ConditionalCommand(
            // coral
            new SequentialCommandGroup(
                new PivotCommand(m_endEffector, 1.3),
                new ElevatorCommand(ElevatorConstants.kL3Height, m_elevator, m_endEffector),
                new PivotCommand(m_endEffector, 1.035),
                new ElevatorCommand(13.8, m_elevator, m_endEffector),
                new PivotCommand(m_endEffector, 0.698), 
                new ElevatorCommand(17.8, m_elevator, m_endEffector)),
            // algae
            new SequentialCommandGroup(
                new ElevatorCommand(0, m_elevator, m_endEffector), 
                new PivotCommand(m_endEffector, 0.5)),
            () -> m_coralMode));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
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
