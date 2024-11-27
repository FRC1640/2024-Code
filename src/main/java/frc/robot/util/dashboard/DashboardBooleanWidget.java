package frc.robot.util.dashboard;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTablesJNI;

public class DashboardBooleanWidget implements DashboardWidget {
    private NetworkTable metadataSubtable;
    private NetworkTable dataSubtable;
    private BooleanPublisher publisher;
    private BooleanSupplier valueSupplier;

    public DashboardBooleanWidget(String name, NetworkTable metadata, NetworkTable data, BooleanSupplier valueSupplier) {
        metadataSubtable = metadata.getSubTable("/" + name);
        dataSubtable = data.getSubTable("/" + name);
        publisher = dataSubtable.getBooleanTopic(name).publish();
        this.valueSupplier = valueSupplier;
    }

    @Override
    public void periodic() {
        publisher.set(valueSupplier.getAsBoolean(), NetworkTablesJNI.now());
    }
}
