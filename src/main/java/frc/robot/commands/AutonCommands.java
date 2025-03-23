package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.coral.CoralL1Command;
import frc.robot.commands.scoring.coral.CoralL4Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.IntakeState;

public class AutonCommands {
	private static ElevatorSubsystem m_elevator;
	private static EndEffectorSubsystem m_endEffector;

	/**
	 * Call before registering commands
	 * 
	 * @param elevator
	 * @param endEffector
	 */
	public static void setSubsystems(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
		m_elevator = elevator;
		m_endEffector = endEffector;
	}

	public static enum Commands {
		GET_CORAL(
				new SequentialCommandGroup(new IntakeOuttakeCoralCommand(m_endEffector, IntakeState.IntakeCoral))),
		PLACE_CORAL_L4(
				new SequentialCommandGroup()),
		RAISE_ELEVATOR_L4(
				new SequentialCommandGroup(new CoralL4Command(m_endEffector, m_elevator))),

		OUTTAKE_CORAL(
				new SequentialCommandGroup(new IntakeOuttakeCoralCommand(m_endEffector, IntakeState.OuttakeCoral)).withTimeout(0.5)),
		LOWER_ELEVATOR(
				new SequentialCommandGroup(new CoralL1Command(m_endEffector, m_elevator))),

		GET_ALGAE(
				new SequentialCommandGroup()),
		PLACE_ALGAE(
				new SequentialCommandGroup());

		private final Command m_command;

		private Commands(Command command) {
			m_command = command;
		}

		public static void registerAutonCommands() {
			for (Commands command : values()) {
				NamedCommands.registerCommand(command.name(), command.m_command);
			}
		}
	}
}
