package frc.lib.drive.Module;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO extends AutoCloseable{
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionMeters;
        public double driveVelocityMetersPerSecond;
        public double driveAppliedVoltage;
        public double driveCurrentAmps;
        public double driveTempCelsius;
        public boolean driveIdleModeIsBrake;;

        public double steerAngleDegrees;
        public double steerRPS;
        public double steerAppliedVoltage;
        public double steerCurrentAmps;
        public double steerTempCelsius;
        public boolean steerIdleModeIsBrake;
        public double steerAngleRadians;
    }

    
    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVoltage(double voltage) {}

    public default void setDrivePercentage(double percentage) {}

    public default void setDriveIdleMode(boolean brake) {}

    public default void setSteerVoltage(double voltage) {}

    public default void setSteerPercentage(double percentage) {}

    public default void setSteerIdleMode(boolean brake) {}
    

    @Override
    public default void close() {}
}
