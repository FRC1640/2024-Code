package frc.robot.util.dashboard;

import java.util.ArrayList;
import java.util.List;

/**
 * Handles the periodics of all {@code MonotriggerCommandSarcophagi} and {@code DitriggerCommandSarcophagi}.
 */
public class CommandSarcophagusPeriodicHandler {
    
    private static List<MonotriggerCommandSarcophagus> monoSarcophagi = new ArrayList<>();
    private static List<DitriggerCommandSarcophagus> diSarcophagi = new ArrayList<>();

    /**
     * Add new {@code MonotriggerCommandSarcophagi} for the handler to manage.
     * 
     * @param sarcophagi New sarcophagi to run periodics of.
     */
    public static void giveSarcophagiMono(MonotriggerCommandSarcophagus ... sarcophagi) {
        for (MonotriggerCommandSarcophagus sarcophagus : sarcophagi) {
            monoSarcophagi.add(sarcophagus);
        }
    }

    /**
     * Add new {@code DitriggerCommandSarcophagi} for the handler to manage.
     * 
     * @param sarcophagi New sarcophagi to run periodics of.
     */
    public static void giveSarcophagiDi(DitriggerCommandSarcophagus ... sarcophagi) {
        for (DitriggerCommandSarcophagus sarcophagus : sarcophagi) {
            diSarcophagi.add(sarcophagus);
        }
    }

    public static void periodic() {
        for (MonotriggerCommandSarcophagus sarcophagus : monoSarcophagi) {
            sarcophagus.periodic();
        }
        for (DitriggerCommandSarcophagus sarcophagus : diSarcophagi) {
            sarcophagus.periodic();
        }
    }
}
