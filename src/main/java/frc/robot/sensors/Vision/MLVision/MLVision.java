package frc.robot.sensors.Vision.MLVision;

import java.util.ArrayList;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.util.Units;
import frc.lib.periodic.PeriodicBase;
import frc.lib.vision.LimelightHelpers;
import frc.lib.vision.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.Constants;

public class MLVision extends PeriodicBase {
    private MLVisionIO io;
    private MLVisionIOInputsAutoLogged inputs = new MLVisionIOInputsAutoLogged();
    private double trigDistance;
   
    private LimelightHelpers.LimelightResults  llresults;
    private LimelightHelpers.LimelightTarget_Detector[] resultsArray;

    private LimelightHelpers.LimelightTarget_Detector targetNote;
    private boolean isTargetNote = false; // has the target note been set

    private int numNotesInView = 0;
    private int numNotesWithinThreshold = 0;



    public MLVision(MLVisionIO io) {
        this.io = io;
        llresults = LimelightHelpers.getLatestResults("limelight-ml");
        resultsArray = llresults.targetingResults.targets_Detector;
    }

    public void periodic() {
        io.updateInputs(inputs);
        
        llresults = LimelightHelpers.getLatestResults("limelight-ml");
        resultsArray = llresults.targetingResults.targets_Detector;
        numNotesInView = resultsArray.length;
        // System.out.println("numNotes in view" + numNotesInView + " == " + resultsArray.length);

        if (isTarget()){
            targetNote = calculateTargetNote();
            isTargetNote = true;
        }
        else{
            isTargetNote = false;
        }

        Logger.processInputs("ML Vision", inputs);

        Logger.recordOutput("MLVision/Distance to note", getDistance());
        
        // all of the tx ty whatever
        Logger.recordOutput("MLVision/Is Target Note Set?", isTargetNote);
        Logger.recordOutput("MLVision/Target TX", getTX());
        Logger.recordOutput("MLVision/Target TA", getTA());
        Logger.recordOutput("MLVision/Target TY", getTY());  
        Logger.recordOutput("MLVision/Notes in View", numNotesInView);
        Logger.recordOutput("MLVision/Notes targeting threshold", numNotesWithinThreshold);        
      
      
        
        io.takeSnapshot(inputs);
    }
    


    // Getters 

    public double getLatency(){
        return inputs.latency;
    }
    public boolean isTarget(){
        return inputs.isTarget;
    }

    public double getTX(){
        if (isTargetNote){   
            return targetNote.tx;
        }
        else{
            return inputs.tx;
        }
    }
    
    public double getTY(){
        if (isTargetNote){   
            return targetNote.ty;
        }
        else{
            return inputs.ty;
        }
    }
    
    public double getTA(){
        if (isTargetNote){   
            return targetNote.ta;
        }
        else{
            return inputs.ta;
        }
    }

    public double getDistance(){
        // Using data points and trigonometry for distance calculations from the
        // Limelight to the object detected.
        if (!inputs.isTarget){
            trigDistance = -1;
        }
        else{
            trigDistance = Units.inchesToMeters( // "d = (h2-h1) / tan(a1+a2)"
                (Constants.MLVisionLimelightConstants.noteHeightInches - Constants.MLVisionLimelightConstants.limelightLensHeight)
                        / Math.tan(Math.toRadians(getTY() + Constants.MLVisionLimelightConstants.limelightAngle)));
        
        }
        return trigDistance;
    }

    public LimelightHelpers.LimelightTarget_Detector calculateTargetNote(){
        
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

        numNotesWithinThreshold = notesWithinThreshold.size();
        // System.out.println("Num notes in threshold: " + numNotesWithinThreshold);

        // find the leftmose note within threshold
        if (notesWithinThreshold.size() > 0){
            for (int i = 0; i < notesWithinThreshold.size(); i++) {
                LimelightHelpers.LimelightTarget_Detector detector = notesWithinThreshold.get(i);
                double tx = detector.tx;
                // System.out.println("TX VAL "+ detector.tx);
                if (tx < minTX) { 
                    minTX = tx;
                    finalNote = detector;
                }
            }
            // System.out.println("TX VAL Fin "+ finalNote.tx);
        }

        return finalNote;
    }

   
    }