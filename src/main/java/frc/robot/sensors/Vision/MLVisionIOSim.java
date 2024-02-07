package frc.robot.sensors.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class MLVisionIOSim implements MLVisionIO {
    DoublePublisher tv, tx, ty, ta;

    public MLVisionIOSim() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ml");
        tx = table.getDoubleTopic("tx").publish();
        tx.set(0);

        ty = table.getDoubleTopic("ty").publish();
        ty.set(0);
        
        ta = table.getDoubleTopic("ta").publish();
        ta.set(0);

        tv = table.getDoubleTopic("tv").publish();
        tv.set(0);
    }

    @Override
    public void updateInputs(MLVisionIOInputs inputs) {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight-ml");

        inputs.tx = networkTable.getEntry("tx").getDouble(0);
        inputs.ty = networkTable.getEntry("ty").getDouble(0);
        inputs.ta = networkTable.getEntry("ta").getDouble(0);

        inputs.isTarget = networkTable.getEntry("tv").getDouble(0) > 0;
    }
}
