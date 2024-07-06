package frc.robot.util.motor;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class FlashLogger {
    
    private List<String[]> flashed;
    public static FlashLogger instance = new FlashLogger();

    public FlashLogger() {
        flashed = new ArrayList<String[]>();
    }

    public static addFlash(SparkMaxConfiguration configuration) {
        
    }
}