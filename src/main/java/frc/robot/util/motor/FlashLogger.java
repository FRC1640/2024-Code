package frc.robot.util.motor;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class FlashLogger {
    
    private List<String[]> flashed;
    private static FlashLogger instance = new FlashLogger();

    public FlashLogger() {
        flashed = new ArrayList<String[]>();
    }

    public static FlashLogger getInstance() {
        return instance;
    }

    public void periodic() {
        for (String[] array : flashed) {
            Logger.recordOutput(array[1], array[0]);
        }
    }

    public void addFlash(SparkMaxConfiguration config) {
        String[] array = { config.getFlashedString(), config.getLogFlashKey() };
        flashed.add(array);
    }
}