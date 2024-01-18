package frc.robot.sensors.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        double[] emptyArray = { 0, 0, 0, 0, 0, 0 };
        inputs.botPose = networkTable.getEntry("botpose_wpiblue").getDoubleArray(emptyArray);
        inputs.isTarget = networkTable.getEntry("tv").getDouble(0) > 0;
    }

}
