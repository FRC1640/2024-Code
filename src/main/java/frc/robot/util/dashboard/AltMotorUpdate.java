package frc.robot.util.dashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;

public class AltMotorUpdate {
    
    private List<GenericEntry> data = new ArrayList<>(2);
    private Function<DoubleSupplier, Command> doubleFunction;
    private BiFunction<DoubleSupplier, BooleanSupplier, Command> biFunctionDouble;

    public AltMotorUpdate(GenericEntry data, Function<DoubleSupplier, Command> consumer) {
        this.data.add(data);
        this.doubleFunction = consumer;
    }
    
    public AltMotorUpdate(GenericEntry dataOne, GenericEntry dataTwo, BiFunction<DoubleSupplier, BooleanSupplier, Command> biFunction) {
        this.data.add(dataOne);
        this.data.add(dataTwo);
        this.biFunctionDouble = biFunction;
    }
    
    public void periodic() {
        if (doubleFunction != null) {
            doubleFunction.accept(data.get(0).getDouble(0));
        }
        else {
            biConsumerDouble.accept(data.get(0).getDouble(0), data.get(1).getBoolean(true));
        }
    }
}