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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveToReef;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final ElevatorSubsystem m_elevator = new ElevatorSubsystem(); // Temporarily commented out to merge
  //private final EndEffectorSubsystem m_endEffector = new EndEffectorSubsystem(() -> 0 /* m_elevator::getHeight */); //TODO: provide supplier // Temporarily commented out

  private final XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(IOConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //TODO: uncomment
    //m_elevator.setEndEffectorSuppliers(m_endEffector::ensureSafeState, m_endEffector::pivotWithinLimits);

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
    
    /* Temporarily commented out to merge
    m_elevator.setDefaultCommand(
        new RunCommand(
            () -> m_elevator.trackPosition(
                MathUtil.applyDeadband(
                    -m_operatorController.getLeftY(),
                    IOConstants.kControllerDeadband)),
            m_elevator));
     */
    
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureBindings() {
    
    new JoystickButton(m_driverController, Button.kStart.value)
        .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

    new JoystickButton(m_driverController, Button.kBack.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d()), m_robotDrive));
    
    /* Temporarily commented out to merge
    new POVButton(m_operatorController, ElevatorConstants.kDPadUp) // Up - L1
        .onTrue(new InstantCommand(
            () -> m_elevator.setHeight(ElevatorConstants.kL1Height)
        ));
    new POVButton(m_operatorController, ElevatorConstants.kDPadRight) // Right - L2
        .onTrue(new InstantCommand(
            () -> m_elevator.setHeight(ElevatorConstants.kL2Height)
        ));
    new POVButton(m_operatorController, ElevatorConstants.kDPadDown) // Down - L3
        .onTrue(new InstantCommand(
            () -> m_elevator.setHeight(ElevatorConstants.kL3Height)
        ));
    new POVButton(m_operatorController, ElevatorConstants.kDPadLeft) // Left - L4
        .onTrue(new InstantCommand(
            () -> m_elevator.setHeight(ElevatorConstants.kL4Height)
        ));
    */

    new JoystickButton(m_driverController, Button.kA.value)
      .whileTrue(new DriveToReef(m_robotDrive, () -> m_driverController.getLeftBumperButton()));
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
    //m_elevator.fastPeriodic(); // Temporarily commented out to merge
    //m_endEffector.fastPeriodic(); // Temporarily commented out
  }
}
