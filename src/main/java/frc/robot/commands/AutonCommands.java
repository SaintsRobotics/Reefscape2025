package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AutonCommands {
	private static ElevatorSubsystem m_elevator;
	private static EndEffectorSubsystem m_endEffector;

	public AutonCommands(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
		m_elevator = elevator;
		m_endEffector = endEffector;
	}

	public static enum Commands {
			// TODO: fill stubs
			GET_CORAL(
							new SequentialCommandGroup()),
			PLACE_CORAL_L1(
							new SequentialCommandGroup()),
			PLACE_CORAL_L2(
							new SequentialCommandGroup()),
			PLACE_CORAL_L3(
							new SequentialCommandGroup()),
			PLACE_CORAL_L4(
							new SequentialCommandGroup()),
			RAISE_ELEVATOR_L4(
							new SequentialCommandGroup()),

			GET_ALGAE(
							new SequentialCommandGroup()),
			PLACE_ALGAE(
							new SequentialCommandGroup()),
							;
			
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
