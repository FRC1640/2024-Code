package frc.robot.util.dashboard;

import java.util.ArrayList;
import java.util.List;

public class MotorUpdatePeriodicHandler {
    
    private static List<MotorUpdate> motorUpdates = new ArrayList<>();

    /**
     * Handles the periodics of all {@code MotorUpdates}.
     */
    public MotorUpdatePeriodicHandler() {
    }

    /**
     * Add new {@code MotorUpdates} for the handler to manage.
     * 
     * @param updates New updates to run periodics of.
     */
    public static void giveMotorUpdates(MotorUpdate ... updates) {
        for (MotorUpdate update : updates) {
            motorUpdates.add(update);
        }
    }

    public static void periodic() {
        for (int i = 0; i < motorUpdates.size(); i++) {
            motorUpdates.get(i).periodic();
        }
    }
}
