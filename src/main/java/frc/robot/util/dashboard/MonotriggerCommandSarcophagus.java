package frc.robot.util.dashboard;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Holds a {@link Command}, recreating it when the value of an associated
 * {@link GenericEntry} has changed but the command is finished.
 */
public class MonotriggerCommandSarcophagus {
  
  private Command spirit;
  private GenericEntry resurrector;
  private Function<DoubleSupplier, Command> recipie;

  /**
   * Holds a {@link Command}, recreating it when the value of an associated
   * {@link GenericEntry} has changed but the command is finished.
   * 
   * @param command {@link Command} to hold.
   * @param entry {@link GenericEntry} on whose change to recreate the command.
   * @param function {@link Function} which returns the desired value of the command, for use when recreating it.
   * Accepts value of GenericEntry.
   */
  public MonotriggerCommandSarcophagus(Command command, GenericEntry entry, Function<DoubleSupplier, Command> function) {
    this.spirit = command;
    this.resurrector = entry;
    this.recipie = function;
  }

  /**
   * Recreates command if GenericEntry has changed and Command has finished.
   */
  public void periodic() {
    if (spirit.isFinished() && resurrector.readQueue().length != 0) {
      spirit = recipie.apply(() -> resurrector.getDouble(0));
      spirit.schedule();
    }
  }
}