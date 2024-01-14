package frc.robot.sensors.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.sensors.Gyro.GyroIO.GyroIOInputs;


public class VisionIOLimelight implements VisionIO {

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");

        inputs.pipeline = networkTable.getEntry("pipeline").getInteger(0);
        // inputs.camMode = networkTable.getEntry("camMode");
        // inputs.ledMode = networkTable.getEntry("ledMode");
        inputs.horizontal = networkTable.getEntry("tx").getDouble(0);
        inputs.vertical = networkTable.getEntry("ta").getDouble(0);
        inputs.targetArea = networkTable.getEntry("ty").getDouble(0);
        inputs.foundTag = networkTable.getEntry("tv").getDouble(0);
        inputs.foundTagBool = (inputs.foundTag != 0) ? true : false;

    }

    @Override
    public long setPipeline(VisionIOInputs inputs){
        return inputs.pipeline;
    }
    
    @Override
    public void setCameraMode(VisionIOInputs inputs){

    }
    
    @Override
    public void setResolution(VisionIOInputs inputs){

    }


}
