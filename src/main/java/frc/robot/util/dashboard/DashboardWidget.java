package frc.robot.util.dashboard;

public interface DashboardWidget {
    public default void periodic() { }

    public default DashboardWidget withSize(int x, int y) { return null; }
}
