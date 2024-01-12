package frc.robot.sensors.Gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs{
        public boolean isConnected = false;
        public boolean isCalibrating = false;
        public double angleDegrees = 0.0;
        public double angularVelocityDegreesPerSecond = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}   

    public default void resetGyro(){}
}
