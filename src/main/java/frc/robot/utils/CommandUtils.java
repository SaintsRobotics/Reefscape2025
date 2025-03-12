package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class CommandUtils {
      /**
     * Creates a three way conditional command. May cause uninteded behavior if both cond1 and cond2 are true
     * @param command1 command to run if cond1 is true
     * @param command2 command to run if and only if cond2 is true
     * @param command3 command to run if and only if cond1 is false and cond2 is false
     * @param cond1 must be mutually exclusive to cond2
     * @param cond2 must be mutually exclusive to cond1
     * @return a three way conditional command
     */
    public static Command generateTripleConditionalCommand(Command command1, Command command2, Command command3, BooleanSupplier cond1, BooleanSupplier cond2) {
        return new ConditionalCommand(
            command1,
            new ConditionalCommand(
                command2,
                command3,
                cond2),
            cond1);
    }
}
