package frc.robot.util.dashboard;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTablesJNI;

public class DashboardBooleanWidget implements DashboardWidget {
    private NetworkTable metadataSubtable;
    // private NetworkTable dataSubtable;
    private BooleanPublisher publisher;
    private BooleanSupplier valueSupplier;

    // I'm ignoring configuration options for now and setting the name of the widget through name in the constructor.
    public DashboardBooleanWidget(String name, NetworkTable metadata, NetworkTable data, BooleanSupplier valueSupplier) {
        metadataSubtable = metadata.getSubTable(name);
        publisher = data.getBooleanTopic(name).publish();
        this.valueSupplier = valueSupplier;
    }

    @Override
    public void periodic() {
        publisher.set(valueSupplier.getAsBoolean(), NetworkTablesJNI.now());
    }
}
