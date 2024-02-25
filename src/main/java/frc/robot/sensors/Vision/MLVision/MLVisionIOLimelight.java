package frc.robot.sensors.Vision.MLVision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.lib.vision.LimelightHelpers;

public class MLVisionIOLimelight implements MLVisionIO {

    @Override
    public void updateInputs(MLVisionIOInputs inputs) {
     NetworkTable MLNetworkTable = NetworkTableInstance.getDefault().getTable("limelight-ml");  // network table for object detection
     inputs.isTarget = MLNetworkTable.getEntry("tv").getDouble(0) > 0;

     inputs.tx = MLNetworkTable.getEntry("tx").getDouble(0); // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     inputs.ty = MLNetworkTable.getEntry("ty").getDouble(0); // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     inputs.ta = MLNetworkTable.getEntry("ta").getDouble(0); // Target Area (0% of image to 100% of image)
    
    }

    @Override
    public void setPipelineIndex(MLVisionIOInputs inputs) {
        LimelightHelpers.setPipelineIndex("limelight-ml", Constants.MLVisionLimelightConstants.mlVisPipeline);
    }
}
