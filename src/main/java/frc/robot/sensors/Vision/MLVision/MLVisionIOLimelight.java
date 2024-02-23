package frc.robot.sensors.Vision.MLVision;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.vision.LimelightHelpers;

public class MLVisionIOLimelight implements MLVisionIO {

    LimelightHelpers.LimelightResults  llresults;
    LimelightHelpers.LimelightTarget_Detector[] resultsArray;

    @Override
    public void updateInputs(MLVisionIOInputs inputs) {
     NetworkTable MLNetworkTable = NetworkTableInstance.getDefault().getTable("limelight-ml");  // network table for object detection
     inputs.isTarget = MLNetworkTable.getEntry("tv").getDouble(0) > 0;

     inputs.tx = MLNetworkTable.getEntry("tx").getDouble(0); // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     inputs.ty = MLNetworkTable.getEntry("ty").getDouble(0); // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     inputs.ta = MLNetworkTable.getEntry("ta").getDouble(0); // Target Area (0% of image to 100% of image)
    
     
     
     llresults = LimelightHelpers.getLatestResults("limelight-ml");
     resultsArray = llresults.targetingResults.targets_Detector;

     inputs.isTargetNote = (resultsArray.length == 0) ? false : true;
    
     // array of total tx values
     inputs.txArray = fillTxArray(); 

     // prioretized values
     inputs.calculatedTx = (inputs.isTarget) ? 0 : calculateTargetNote().tx;;
     inputs.calculatedTy = (inputs.isTarget) ? 0 : calculateTargetNote().ty;
     inputs.calculatedTa = (inputs.isTarget) ? 0 : calculateTargetNote().ta;
    
    }

    // fills an arraylist with doubles of every tx value of every note visible
    private ArrayList<Double> fillTxArray() {  
        ArrayList<Double> txArray = new ArrayList<Double>();
        for (LimelightHelpers.LimelightTarget_Detector detector : resultsArray) {
          txArray.add(detector.tx);  
        }
        return txArray;
    }

    private LimelightHelpers.LimelightTarget_Detector calculateTargetNote(){
        
        ArrayList<LimelightHelpers.LimelightTarget_Detector> notesArray = new ArrayList<LimelightHelpers.LimelightTarget_Detector>() ;

        double maxTa = -1;        
        int indexWithLargestTa = -1;

        double threshold = 1;

        ArrayList<LimelightHelpers.LimelightTarget_Detector> notesWithinThreshold = new ArrayList<LimelightHelpers.LimelightTarget_Detector>() ;
        double minTX = 27;
        LimelightHelpers.LimelightTarget_Detector finalNote = new LimelightHelpers.LimelightTarget_Detector();

        // Get rid of ones with class != note AKA fill notesArray list
        for (int i = 0; i < resultsArray.length; i++) {
            LimelightHelpers.LimelightTarget_Detector detector = resultsArray[i];//.getJSONObject(i);
           
            if (detector.className.equals("note")) { 
                notesArray.add(detector);
            }  
        }

        // Find note with the largest TA
        for (int i = 0; i < notesArray.size(); i++) {
            LimelightHelpers.LimelightTarget_Detector detector = notesArray.get(i);           
             double ta = detector.ta;
           
            if (ta > maxTa) { 
                //update the max with the note currently being read
                maxTa = ta;
                indexWithLargestTa = i;
            }  
        }
        
        //fill feasible notes array (notes within threshold)
        for (LimelightHelpers.LimelightTarget_Detector detector : notesArray) {
            if (detector.ta > maxTa-threshold) { 
                notesWithinThreshold.add(detector);
            }  
        }

        //numNotesWithinThreshold = notesWithinThreshold.size();
        // System.out.println("Num notes in threshold: " + numNotesWithinThreshold);

        // find the leftmose note within threshold
        if (notesWithinThreshold.size() > 0){
            for (int i = 0; i < notesWithinThreshold.size(); i++) {
                LimelightHelpers.LimelightTarget_Detector detector = notesWithinThreshold.get(i);
                double tx = detector.tx;
                //System.out.println("TX VAL "+ detector.tx);
                if (tx < minTX) { 
                    minTX = tx;
                    finalNote = detector;
                }
            }
            //System.out.println("TX VAL Fin "+ finalNote.tx);
        }

        return finalNote;
    }
}
