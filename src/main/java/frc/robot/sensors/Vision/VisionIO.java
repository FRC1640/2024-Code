package frc.robot.sensors.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public double latency;
        public Pose2d aprilTagPose;
        public boolean isTarget;
    }
    public default void updateInputs(VisionIOInputs inputs) {
    }
}
