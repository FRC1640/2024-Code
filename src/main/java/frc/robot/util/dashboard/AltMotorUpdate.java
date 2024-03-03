package frc.robot.util.dashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;

public class AltMotorUpdate {
    
    private List<GenericEntry> data = new ArrayList<>(2);
    private Function<DoubleSupplier, Command> doubleFunction;
    private BiFunction<DoubleSupplier, BooleanSupplier, Command> biFunctionDouble;
    private Command shelteredCommand;

    /**
     * Updates a motor or motors in a subsystem to states set from Shuffleboard in test mode. Will periodically
     * set the percent output of a motor to that which is published to NetworkTables from Shuffleboard.
     * 
     * @param data {@code GenericEntry} containing the desired percent output.
     * @param doubleFunction {@code Function<DoubleSupplier, Command>} that sets the percent output from a subsystem.
     */
    public AltMotorUpdate(GenericEntry data, Function<DoubleSupplier, Command> doubleFunction) {
        this.data.add(data);
        this.doubleFunction = doubleFunction;
        this.shelteredCommand = this.doubleFunction.apply(() -> this.data.get(0).getDouble(0));
        shelteredCommand.schedule();
    }
    
    /**
     * Updates a motor or motors in a subsystem to states set from Shuffleboard in test mode. Will periodically
     * set the percent output of a motor to that which is published to NetworkTables from Shuffleboard and turn
     * motor limits on or off based on Shuffleboard data.
     * 
     * @param data1 {@code GenericEntry} containing the desired percent output.
     * @param data2 {@code GenericEntry} containing the desired limit state.
     * @param doubleFunction {@code BiFunction<DoubleSupplier, BooleanSupplier, Command>} that sets the percent
     * output from a subsystem and toggles limits to match the given state.
     */
    public AltMotorUpdate(GenericEntry data1, GenericEntry data2,
            BiFunction<DoubleSupplier, BooleanSupplier, Command> biFunction) {
        this.data.add(data1);
        this.data.add(data2);
        this.biFunctionDouble = biFunction;
        this.shelteredCommand = this.biFunctionDouble.apply(() -> this.data.get(0).getDouble(0),
                () -> this.data.get(1).getBoolean(true));
        shelteredCommand.schedule();
        System.out.println("Constructed with limits!");
    }

    /**
     * Gets the {@code Command} created by the constructor.
     * 
     * @return {@code Command}.
     */
    public Command publishCommand() {
        return shelteredCommand;
    }
    
    ///
    /// Resurrects this object's {@code Command} if this object's
    /// {@code GenericEntries} have changed but the command no longer exists.
    ///
    public void periodic() {
        // if (shelteredCommand.isFinished()) {
        //     try (GenericEntry entry = data.get(0)) {
        //         try (GenericEntry entry1 = data.get(data.size() - 1)) {
        //             if (entry.readQueue().length != 0 || entry.readQueue().length != 0) {
        //                 if (doubleFunction != null) {
        //                     shelteredCommand = doubleFunction.apply(() -> data.get(0).getDouble(0));
        //                     System.out.println("Complete.");
        //                 } else {
        //                     shelteredCommand = biFunctionDouble.apply(() -> data.get(0).getDouble(0),
        //                        () -> data.get(1).getBoolean(false));
        //                     System.out.println("Over.");
        //                 }
        //                 System.out.println("Ready.");
        //             }
        //             shelteredCommand.schedule();
        //             System.out.println("Done.");
        //         }
        //     }
        // }
        // System.out.println("Finished.");
    }
}