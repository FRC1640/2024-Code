package frc.lib.sysid;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CreateSysidCommand {
    public static Command createCommand(Function<SysIdRoutine.Direction, Command>  quasistaticCommand, Function<SysIdRoutine.Direction, Command>  dynamicCommand,
            String commandName, CommandXboxController controller, Runnable stopMotors) {
        Command sysIdCommand = new SequentialCommandGroup(
                quasistaticCommand.apply(SysIdRoutine.Direction.kForward)
                        .finallyDo(()->stopMotors.run()).until(()->controller.a().getAsBoolean()),
                new WaitUntilCommand(()->controller.b().getAsBoolean()),
                quasistaticCommand.apply(SysIdRoutine.Direction.kReverse)
                        .finallyDo(()->stopMotors.run()).until(()->controller.a().getAsBoolean()),
                new WaitUntilCommand(()->controller.b().getAsBoolean()),
                dynamicCommand.apply(SysIdRoutine.Direction.kForward)
                        .finallyDo(()->stopMotors.run()).until(()->controller.a().getAsBoolean()),
                new WaitUntilCommand(()->controller.b().getAsBoolean()),
                dynamicCommand.apply(SysIdRoutine.Direction.kReverse)
                       .finallyDo(()->stopMotors.run()) .until(()->controller.a().getAsBoolean()));
        sysIdCommand.setName(commandName);
        return sysIdCommand;
    }
}
