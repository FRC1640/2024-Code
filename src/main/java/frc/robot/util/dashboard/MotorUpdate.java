package frc.robot.util.dashboard;

import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.GenericEntry;

public class MotorUpdate {
    
    private GenericEntry motorSpeed;
    private DoubleConsumer setter;

    /**
     * Periodically updates the percent output of a motor or group of motors.
     * 
     * @param motorSpeed NetworkTables {@code GenericEntry} from which to get the speed.
     * @param setter {@code DoubleConsumer} from a subsystem to use to set the motor speeds.
     */
    public MotorUpdate(GenericEntry motorSpeed, DoubleConsumer setter) {
        this.motorSpeed = motorSpeed;
        this.setter = setter;
    }

    /**
     * Runs the {@code DoubleConsumer} with the speed from the {@code GenericEntry}.
     */
    public void periodic() {
        setter.accept(motorSpeed.getDouble(0));
    }
}