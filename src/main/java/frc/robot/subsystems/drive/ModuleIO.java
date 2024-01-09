package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO extends AutoCloseable{
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionMeters;
        public double driveVelocityMetersPerSecond;
        public double driveAppliedVoltage;
        public double driveCurrentAmps;
        public double driveTempCelsius;
        public boolean driveIdleModeIsBreak;
        public boolean driveIsFlipped;

        public double steerAbsoluteAngleDegrees;
        public double steerRelativeAngleDegrees;
        public double steerVelocityRadiansPerSecond;
        public double steerAppliedVoltage;
        public double steerCurrentAmps;
        public double steerTempCelsius;
        public boolean steerIdleModeIsBrake;
    }

    
    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVoltage(double voltage) {}

    public default void setDrivePercentage(double percentage) {}

    public default void setDriveIdleMode(boolean brake) {}

    public default void setSteerVoltage(double voltage) {}

    public default void setSteerPercentage(double percentage) {}

    public default void setSteerIdleMode(boolean brake) {}

    public default void stopAllMotors() {}

    

    @Override
    public default void close() {}
}
