package frc.robot.sensors.Vision.MLVision;

import org.littletonrobotics.junction.AutoLog;

public interface MLVisionIO {
    
    @AutoLog

    public static class MLVisionIOInputs {
        public double latency;
        public boolean isTarget;
        public double tx;
        public double ty;
        public double ta;
        
    }
    public default void updateInputs(MLVisionIOInputs inputs) {
    }

    public default void takeSnapshot(MLVisionIOInputs inputs){
    }
}
