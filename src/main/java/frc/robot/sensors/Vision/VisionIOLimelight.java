package frc.robot.sensors.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class VisionIOLimelight implements VisionIO {

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        double[] temp = {0, 0, 0 ,0,0,0};
        inputs.botPose = networkTable.getEntry("botpose").getDoubleArray(temp);

        //inputs.pipeline = networkTable.getEntry("pipeline").getInteger(0);
        //inputs.camMode = networkTable.getEntry("camMode");
        //inputs.ledMode = networkTable.getEntry("ledMode");
        //inputs.horizontal = networkTable.getEntry("tx").getDouble(0);
        //inputs.vertical = networkTable.getEntry("ta").getDouble(0);
        //inputs.targetArea = networkTable.getEntry("ty").getDouble(0);
        //inputs.foundTag = networkTable.getEntry("tv").getDouble(0);
        //inputs.foundTagBool = (inputs.foundTag != 0) ? true : false;

        //inputs.horizontalResolution = networkTable.getEntry("pipeline").getInteger(0);
        //inputs.verticalResolution = networkTable.getEntry("pipeline").getInteger(0);
        
        
        //inputs.tl = networkTable.getEntry("tl").getDouble(0);
        //inputs.cl = networkTable.getEntry("cl").getDouble(0);


        

    }

    //@Override
    //public void setPipeline(VisionIOInputs inputs){}
    
    //@Override
    //public void setCameraMode(VisionIOInputs inputs){}


}
