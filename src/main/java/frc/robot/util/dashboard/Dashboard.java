package frc.robot.util.dashboard;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Dashboard {
    private static Dashboard instance;
    private static NetworkTable dashboardTable;
    private static NetworkTable metadata;
    private static NetworkTable data;
    private static ArrayList<DashboardWidget> widgets;

    public Dashboard() {
        instance = this;
        dashboardTable = NetworkTableInstance.getDefault().getTable("Dashboard");
        metadata = dashboardTable.getSubTable("/Metadata");
        data = dashboardTable.getSubTable("/Values");
        widgets = new ArrayList<>();
    }
    
    public static DashboardBooleanWidget addBoolean(String title, BooleanSupplier valueSupplier) {
        DashboardBooleanWidget newWidget = new DashboardBooleanWidget(title, metadata, data, valueSupplier);
        widgets.add(newWidget);
        return newWidget;
    }

    public static Dashboard instance() {
        return instance;
    }

    public static void periodic() {
        for (DashboardWidget widget : widgets) {
            widget.periodic();
        }
    }
}
