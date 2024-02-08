package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double leftSpeedPercent = 0.0;
        public double leftAppliedVoltage = 0.0;
        public double leftCurrentAmps = 0.0;
        public double leftTempCelcius = 0.0;
        public double leftClimberPositionDegrees = 0.0;

        public double rightSpeedPercent = 0.0;
        public double rightAppliedVoltage = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightTempCelcius = 0.0;
        public double rightClimberPositionDegrees = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs){

    }

    public default void setLeftVoltage(double lVoltage){

    }

    public default void setRightVoltage(double rVoltage){

    }

    public default void setLeftSpeedPercent(double lSpeed){
        
    }

    public default void setRightSpeedPercent(double rSpeed){

    }
}
