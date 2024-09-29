package frc.robot.sensors.Vision.MLVision;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.vision.LimelightHelpers;
import frc.lib.vision.LimelightHelpers.LimelightResults;

public class MLVisionIOLimelight implements MLVisionIO {

    LimelightHelpers.LimelightResults llresults;
    LimelightHelpers.LimelightTarget_Detector[] resultsArray;

    @Override
    public void updateInputs(MLVisionIOInputs inputs) {
        
        NetworkTable MLNetworkTable = NetworkTableInstance.getDefault().getTable("limelight-ml"); // network table for
                                                                                                  // object detection
        inputs.isTarget = MLNetworkTable.getEntry("tv").getDouble(0) > 0;

        inputs.tx = MLNetworkTable.getEntry("tx").getDouble(0); // Horizontal Offset From Crosshair To Target (LL1: -27
                                                                // degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
        inputs.ty = MLNetworkTable.getEntry("ty").getDouble(0); // Vertical Offset From Crosshair To Target (LL1: -20.5
                                                                // degrees to 20.5 degrees / LL2: -24.85 to 24.85
                                                                // degrees)
        inputs.ta = MLNetworkTable.getEntry("ta").getDouble(0); // Target Area (0% of image to 100% of image)

        llresults = LimelightHelpers.getLatestResults("limelight-ml");

        
        resultsArray = llresults.targetingResults.targets_Detector;
        inputs.numVisibleNotes = resultsArray.length;

        inputs.allTx = Arrays.stream(resultsArray).mapToDouble((x) -> x.tx).toArray();
        inputs.allTy = Arrays.stream(resultsArray).mapToDouble((x) -> x.ty).toArray();

        inputs.confidence = Arrays.stream(resultsArray).mapToDouble((x)->x.confidence).toArray();

    }
}
