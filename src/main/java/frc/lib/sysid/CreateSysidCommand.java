package frc.lib.sysid;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CreateSysidCommand {
    public static Command createCommand(Function<SysIdRoutine.Direction, Command>  quasistaticCommand, Function<SysIdRoutine.Direction, Command>  dynamicCommand,
            String commandName, BooleanSupplier cancelCondition, BooleanSupplier startNextCondition) {
        Command sysIdCommand = new SequentialCommandGroup(
                quasistaticCommand.apply(SysIdRoutine.Direction.kForward)
                        .until(cancelCondition),
                new WaitUntilCommand(startNextCondition),
                quasistaticCommand.apply(SysIdRoutine.Direction.kReverse)
                        .until(cancelCondition),
                new WaitUntilCommand(startNextCondition),
                dynamicCommand.apply(SysIdRoutine.Direction.kForward)
                        .until(cancelCondition),
                new WaitUntilCommand(startNextCondition),
                dynamicCommand.apply(SysIdRoutine.Direction.kReverse)
                        .until(cancelCondition));
        sysIdCommand.setName(commandName);
        return sysIdCommand;
    }
}
