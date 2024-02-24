package frc.robot.sensors.Vision.MLVision;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.vision.LimelightHelpers;

public interface MLVisionIO {
    
    @AutoLog

    public static class MLVisionIOInputs {
        public double latency;
        public boolean isTarget;
        
        // default network tables outputs (based on greatest TA)
        public double tx;
        public double ty;
        public double ta;

        // array of total tx values
        //Double[] txArray;
        public int numVisibleNotes;


        // prioretized values
        public boolean isTargetNote;
        public double calculatedTx;
        public double calculatedTy;
        public double calculatedTa;

        
    }
    public default void updateInputs(MLVisionIOInputs inputs) {
    }

    public default void takeSnapshot(MLVisionIOInputs inputs){
    }
}
