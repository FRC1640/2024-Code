package frc.robot.util.motor;

import java.util.Map;

public class FlashLogger {
    
    private Map<String, Boolean> flashed;
    public static FlashLogger instance = new FlashLogger();

    public FlashLogger() {
        flashed = new Map<String,Boolean>() {};
    }

    public static addFlash(SparkMaxConfiguration configuration) {
        
    }
}
