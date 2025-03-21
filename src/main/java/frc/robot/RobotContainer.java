// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.AutonCommands;
import frc.robot.commands.IntakeOuttakeCoralCommand;
import frc.robot.commands.DriveToReef;
import frc.robot.commands.ElevatorSemiAutomaticDriveCommand;
import frc.robot.commands.HapticCommand;
import frc.robot.commands.scoring.BargeFlipCommand;
import frc.robot.commands.scoring.L1Command;
import frc.robot.commands.scoring.L2Command;
import frc.robot.commands.scoring.L3Command;
import frc.robot.commands.scoring.L4Command;
import frc.robot.commands.scoring.algae.AlgaeBargeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.IntakeState;
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
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

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

    AutoBuilder.configure(m_robotDrive::getPose, (pose) -> m_robotDrive.resetOdometry(pose),
            () -> m_robotDrive.getRobotRelativeSpeeds(),
            (speeds) -> m_robotDrive.autonDrive(speeds),
            new PPHolonomicDriveController(AutonConstants.kTranslationConstants, AutonConstants.kRotationConstants),
            AutonConstants.kBotConfig,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, m_robotDrive);

    AutonCommands.setSubsystems(m_elevator, m_endEffector);
    AutonCommands.Commands.registerAutonCommands();

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

    m_ledSubsystem.setDefaultCommand(m_ledSubsystem.coralModeCommand(() -> m_coralMode, m_endEffector::isHolding).withTimeout(0.25));
}

/**
 * runs on teleop and auton init
 */
public void initSubsystems() {
    // cancel commands
    new InstantCommand(() -> {}, m_elevator, m_endEffector).schedule();

    // m_elevator.zeroPosition();
    m_elevator.setHeight(m_elevator.getCurrentHeight());
    m_endEffector.pivotTo(m_endEffector.getPivotPosition());
}

  /**
   *  Driver Controls:
   * 
   *    Driving:
   *      left axis X/Y                         axis  Translation
   *      right axis X                          axis  Rotation
   *      start                                 press Reset heading
   *      back                                  press Reset position
   *      A (left bumper pressed)               held  Align to reef
   *      right bumper                          held  Slow mode
   * 
   *    End Effector:
   *      left trigger                          held  Intake
   *      right trigger                         held  Outtake
   *      
   *  Operator Controls:
   * 
   *    End Effector:
   *      A                                     held  Intake coral
   *      X                                     held  Outtake coral
   *      right axis Y (Y button pressed)       axis  Pivot control
   *      right bumper                          press Algae mode
   *      left bumper                           press Coral mode
   *      
   *    Elevator:
   *      back                                  press Zero elevator height (see 1)
   *      start                                 press Increment elevator height (see 2)
   *      left axis Y (B and Y button pressed)  axis  Manual elevator control
   *      left axis Y (Y button pressed)        axis  Semi-automatic elevator control
   *      Dpad up                               press L1 or source
   *      Dpad right                            press L2
   *      Dpad down                             press L3
   *      Dpad left                             press L4 or barge
   * 
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
      // -------- driving bindings -------- //

    // driver reset heading
    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    // driver reset odometry
    new JoystickButton(m_driverController, Button.kBack.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d()), m_robotDrive));

    // new JoystickButton(m_driverController, Button.kB.value)
    //     .onTrue(m_ledSubsystem.tripleBlink(0, 128, 0, "green"));

    // driver drive to reef
    new JoystickButton(m_driverController, Button.kA.value)
       .whileTrue(new DriveToReef(m_robotDrive, () -> m_driverController.getLeftBumperButton()))
        .whileTrue(new RepeatCommand(m_ledSubsystem.tripleBlink(128, 0, 0, "auto")));


    // -------- end effector bindings -------- //

    // operator hold to intake coral (or weakly outtake algae)
    new JoystickButton(m_operatorController, Button.kA.value)
      .onTrue(new InstantCommand(m_endEffector::forceCoral, m_endEffector))
      .onFalse(new InstantCommand(m_endEffector::stopEffector, m_endEffector));

    // operator hold to outtake coral (or weakly intake algae)
    new JoystickButton(m_operatorController, Button.kX.value)
      .onTrue(new RunCommand(m_endEffector::reverseCoral, m_endEffector))
      .onFalse(new InstantCommand(m_endEffector::stopEffector, m_endEffector));
    
    // operator pivot manual control
    new Trigger(() -> (MathUtil.applyDeadband(m_operatorController.getRightY(), IOConstants.kControllerDeadband) != 0) && m_operatorController.getYButton())
            .whileTrue(new RunCommand(() -> {
                m_endEffector.setSpeed(-m_operatorController.getRightY() * IOConstants.kPivotAxisScalar); // no need to apply deadband here because of trigger
            }, m_endEffector));

    // operator algae mode
    new JoystickButton(m_operatorController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> {
          m_coralMode = false;
          SmartDashboard.putBoolean("coral mode", m_coralMode);
        }));

    // operator coral mode
    new JoystickButton(m_operatorController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> {
          m_coralMode = true;
          SmartDashboard.putBoolean("coral mode", m_coralMode);
        }));

    // driver intake (use m_coralMode)
    new Trigger(() -> m_driverController.getLeftTriggerAxis() > IOConstants.kControllerDeadband)
        .whileTrue(new ConditionalCommand(
          // coral
          new SequentialCommandGroup(
            new IntakeOuttakeCoralCommand(m_endEffector, IntakeState.IntakeCoral),
            new ParallelCommandGroup(
                new HapticCommand(m_driverController),
                new InstantCommand(() -> m_ledSubsystem.tripleBlink(0, 255, 0, "Intookened"), m_ledSubsystem),
                new HapticCommand(m_operatorController))),
          // algae
          new StartEndCommand(m_endEffector::intakeAlgae, m_endEffector::stopEffector),
          () -> m_coralMode));

    // driver outtake
    new Trigger(() -> m_driverController.getRightTriggerAxis() > IOConstants.kControllerDeadband)
        .whileTrue(new ConditionalCommand(
          // coral
          new IntakeOuttakeCoralCommand(m_endEffector, IntakeState.OuttakeCoral),
          // algae
          new StartEndCommand(m_endEffector::outtakeAlgae, m_endEffector::stopEffector),
          () -> m_coralMode));

    // driver barge flip
    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(new SequentialCommandGroup(
            new AlgaeBargeCommand(m_endEffector, m_elevator),
            new BargeFlipCommand(m_endEffector),
            new ParallelCommandGroup(
                new HapticCommand(m_driverController),
                new HapticCommand(m_operatorController)
        )));

    // -------- elevator bindings -------- //
    // operator zero elevator position
    new JoystickButton(m_operatorController, Button.kBack.value)
            .onTrue(new InstantCommand(() -> m_elevator.zeroPosition(), m_elevator));

    // operator zero elevator position with offset of 5 inches (allows driving down without much restriction, useful for worst case lockup)
    new JoystickButton(m_operatorController, Button.kStart.value)
            .onTrue(new InstantCommand(() -> m_elevator.zeroPosition(5), m_elevator));

    // operator full manual elevator
    new JoystickButton(m_operatorController, Button.kB.value)
            .and(() -> MathUtil.applyDeadband(m_operatorController.getLeftY(), IOConstants.kControllerDeadband) != 0)
            .and(() -> m_operatorController.getYButton())
            .whileTrue(new RunCommand(() -> {
                m_elevator.setSpeed(-m_operatorController.getLeftY() * IOConstants.kElevatorAxisScalar); // no need to apply deadband here because of trigger
            }, m_elevator));

    // operator semi manual elevator
    new JoystickButton(m_operatorController, Button.kB.value).negate()
            .and(() -> m_operatorController.getYButton())
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

    // operator POV buttons
    new POVButton(m_operatorController, IOConstants.kDPadUp) // Up - L1
        .onTrue(
            new SequentialCommandGroup(
                new L1Command(m_endEffector, m_elevator, () -> m_coralMode), 
                new HapticCommand(m_driverController)));

    new POVButton(m_operatorController, IOConstants.kDPadRight) // Right - L2
        .onTrue(new SequentialCommandGroup(
            new L2Command(m_endEffector, m_elevator, () -> m_coralMode), 
            new HapticCommand(m_driverController)));

    new POVButton(m_operatorController, IOConstants.kDPadDown) // Down - L3
        .onTrue(new SequentialCommandGroup(
            new L3Command(m_endEffector, m_elevator, () -> m_coralMode), 
            new HapticCommand(m_driverController)));

    new POVButton(m_operatorController, IOConstants.kDPadLeft) // Left - L4
        .onTrue(new SequentialCommandGroup(
            new L4Command(m_endEffector, m_elevator, () -> m_coralMode), 
            new HapticCommand(m_driverController)));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();

    // An example command will be run in autonomous
    // return new SequentialCommandGroup(
    //     new ParallelDeadlineGroup(new WaitCommand(2), new RunCommand(() -> m_robotDrive.drive(1, 0, 0, false), m_robotDrive)),
    //     new ParallelDeadlineGroup(new WaitCommand(0.1), new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive)),
    //     new ParallelDeadlineGroup(new WaitCommand(3), new InstantCommand(() -> m_endEffector.outtakeCoral(), m_endEffector)),
    //     new InstantCommand((() -> m_endEffector.stopEffector()), m_endEffector));

    // Simple drive forwards
    // return new SimpleDriveForwards(m_robotDrive, 2, 1.5);

    // Center drive forwards and score
    // return new DriveForwardsL1(m_robotDrive, m_endEffector, 3, 1);

    // Blue left
    // return new BlueLeft(m_robotDrive, m_elevator, m_endEffector);

    // Red right
    // return new RedRight(m_robotDrive, m_elevator, m_endEffector);



    // return new ParallelDeadlineGroup(new WaitCommand(3), new RunCommand(() -> m_robotDrive.drive(1, 0, 0, false), m_robotDrive));

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
