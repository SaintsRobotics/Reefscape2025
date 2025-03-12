package frc.robot.utils;

import java.security.InvalidParameterException;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class CommandUtils {
    /**
     * Creates a conditional command with N commands. N must be greater than 2
     * @param commands The list of commands with length N
     * @param conditions The list of conditions. Condition i being true means command i will run. The <b>last</b> command in the list has priority. Must have length N
     */
    public static Command generateNConditionalCommand(List<Command> commands, List<BooleanSupplier> conditions) {
        final int N = commands.size();
        if (N < 2) {
            throw new InvalidParameterException("N must be greater than 2");
        }

        ConditionalCommand[] commandArray = new ConditionalCommand[N-1];
        commandArray[0] = new ConditionalCommand(commands.get(1), commands.get(0), conditions.get(1));

        // iterate in batches of two
        for (int i = 2; i < N; i++) {
            commandArray[i-1] = new ConditionalCommand(commands.get(i), commandArray[i-2], conditions.get(i));
        }

        return commandArray[N-2];
    }
}
