package frc.robot.sensors.Vision.MLVision;

import org.littletonrobotics.junction.AutoLog;

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


        public double[] allTx;
        public double[] allTy;

        public double[] width;
        public double[] height;

        public double[] pts;

        public double[] confidence;
    }
    public default void updateInputs(MLVisionIOInputs inputs) {
    }

    public default void takeSnapshot(MLVisionIOInputs inputs){
    }
}
