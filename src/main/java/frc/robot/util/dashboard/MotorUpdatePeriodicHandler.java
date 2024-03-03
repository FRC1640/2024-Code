package frc.robot.util.dashboard;

import java.util.ArrayList;
import java.util.List;

/**
 * Handles the periodics of all {@code AltMotorUpdates}.
 */
public class MotorUpdatePeriodicHandler {
    
    private static List<AltMotorUpdate> motorUpdates = new ArrayList<>();

    /**
     * Add new {@code MotorUpdates} for the handler to manage.
     * 
     * @param updates New updates to run periodics of.
     */
    public static void giveMotorUpdates(AltMotorUpdate ... updates) {
        for (AltMotorUpdate update : updates) {
            motorUpdates.add(update);
        }
    }

    public static void periodic() {
        for (int i = 0; i < motorUpdates.size(); i++) {
            motorUpdates.get(i).periodic();
        }
    }
}
