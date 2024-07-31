package frc.robot.sensors.Vision.MLVision;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class MLVisionIOSim implements MLVisionIO {
    DoublePublisher tv, tx, ty, ta, isTargetNote, calculatedTx, calculatedTy, calculatedTa;

    DoubleArrayPublisher alltx, allty;

    public MLVisionIOSim() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-ml");

        alltx = table.getDoubleArrayTopic("alltx").publish();
        alltx.set(new double[]{0});

        allty = table.getDoubleArrayTopic("allty").publish();
        allty.set(new double[]{0});

        tx = table.getDoubleTopic("tx").publish();
        tx.set(0);

        ty = table.getDoubleTopic("ty").publish();
        ty.set(0);
        
        ta = table.getDoubleTopic("ta").publish();
        ta.set(0);

        tv = table.getDoubleTopic("tv").publish();
        tv.set(0);

        isTargetNote = table.getDoubleTopic("isTargetNote").publish();
        isTargetNote.set(0);
       
        calculatedTx = table.getDoubleTopic("calculatedTx").publish();
        calculatedTx.set(0);
        
        calculatedTy = table.getDoubleTopic("calculatedTy").publish();
        calculatedTy.set(0);
        
        calculatedTa = table.getDoubleTopic("calculatedTa").publish();
        calculatedTa.set(0);
    }

    @Override
    public void updateInputs(MLVisionIOInputs inputs) {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight-ml");

        inputs.tx = networkTable.getEntry("tx").getDouble(0);
        inputs.ty = networkTable.getEntry("ty").getDouble(0);
        inputs.ta = networkTable.getEntry("ta").getDouble(0);

        inputs.isTarget = networkTable.getEntry("tv").getDouble(0) > 0;

        inputs.isTargetNote = networkTable.getEntry("isTargetNote").getDouble(0)> 0;
        inputs.calculatedTx = networkTable.getEntry("calculatedTx").getDouble(0);
        inputs.calculatedTy = networkTable.getEntry("calculatedTy").getDouble(0);
        inputs.calculatedTa = networkTable.getEntry("calculatedTa").getDouble(0);


        inputs.allTx = networkTable.getEntry("alltx").getDoubleArray(new double[]{0});
        inputs.allTy = networkTable.getEntry("allty").getDoubleArray(new double[]{0});
    }
}
