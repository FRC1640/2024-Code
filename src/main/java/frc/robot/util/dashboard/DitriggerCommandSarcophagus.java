package frc.robot.util.dashboard;

import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Holds a {@link Command}, recreating it when the value of an associated
 * {@link GenericEntry} has changed but the command is finished.
 * Different from {@link MonotriggerCommandSarcophagus} in that this
 * class reconstructs the command using a
 * {@code BiFunction<DoubleSupplier, BooleanSupplier, Command>} and
 * takes two GenericEntries--either of which can trigger the reconstruction
 * of the Command.
 */
public class DitriggerCommandSarcophagus {
  
  private Command spirit;
  private GenericEntry resurrectorDouble;
  private GenericEntry resurrectorBoolean;
  private BiFunction<DoubleSupplier, BooleanSupplier, Command> recipie;

  /**
   * Holds a {@link Command}, recreating it when the value of one of the associated
   * {@link GenericEntry}s has changed but the command is finished.
   * 
   * @param command {@link Command} to hold.
   * @param entry1 {@link GenericEntry} containing a {@code double}, on whose change to recreate the command.
   * @param entry2 GenericEntry containing a {@code boolean}, on whose change to recreate the command.
   * @param function {@link BiFunction} which returns the desired value of the command, for use when recreating it.
   * Accepts values from GenericEntries.
   */
  public DitriggerCommandSarcophagus(Command command, GenericEntry entry1, GenericEntry entry2,
      BiFunction<DoubleSupplier, BooleanSupplier, Command> function) {
    this.spirit = command;
    this.resurrectorDouble = entry1;
    this.resurrectorBoolean = entry2;
    this.recipie = function;
  }

  /**
   * Recreates command if either GenericEntry has changed and Command has finished.
   */
  public void periodic() {
    if (spirit.isFinished()) {
      if (resurrectorDouble.readQueue().length != 0 || resurrectorBoolean.readQueue().length != 0) {
        spirit = recipie.apply(() -> resurrectorDouble.getDouble(0), () -> resurrectorBoolean.getBoolean(false));
        spirit.schedule();
      }
    }
  }
}