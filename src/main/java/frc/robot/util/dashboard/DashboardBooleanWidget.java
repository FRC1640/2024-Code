package frc.robot.util.dashboard;

import java.util.ArrayList;
import java.util.Vector;
import java.util.function.BooleanSupplier;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.Publisher;

public class DashboardBooleanWidget implements DashboardWidget {
    private NetworkTable metadataSubtable;
    // private ArrayList<Publisher> metadataPublishers;
    // private Vector<Integer> size = new Vector<>();
    private BooleanPublisher publisher;
    private BooleanSupplier valueSupplier;

    // Ignoring configuration options for now and setting the name of the widget through name in the constructor.
    public DashboardBooleanWidget(String name, NetworkTable metadata, NetworkTable data, BooleanSupplier valueSupplier) {
        metadataSubtable = metadata.getSubTable(name);
        // metadataPublishers = new ArrayList<>();
        publisher = data.getBooleanTopic(name).publish();
        this.valueSupplier = valueSupplier;
    }

    @Override
    public void periodic() {
        publisher.set(valueSupplier.getAsBoolean(), NetworkTablesJNI.now());
        // for (Publisher publisher : metadataPublishers) {
        //     publisher.set(, NetworkTablesJNI.now());
        // }
    }

    // @Override
    // public DashboardBooleanWidget withSize(int x, int y) {
    //     Publisher publisher = metadataSubtable.getIntegerArrayTopic("size").publish();
    //     metadataPublishers.add(publisher);
    //     return this;
    // }
}
