package frc.robot.sensors.Vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public double[] botPose = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };// (x,y,z in meters, roll, pitch, yaw in
                                                                        // degrees, and cl+tl (total latency)).
        public boolean isTarget;
    }
    public default void updateInputs(VisionIOInputs inputs) {
    }
}
