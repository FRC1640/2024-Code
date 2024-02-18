package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.TargetingConstants;

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
    /**
     * Modifies the inputted speed so as to not move out of limits
     * 
     * @param pos the current position.
     * @param speed the base speed to clamp.
     * @return clamped speed.
     */
    public default double clampSpeeds(double pos, double speed) {
        double speedClamped = speed;
        if (pos < ClimberConstants.lowerLimit) {
            speedClamped = Math.max(speed, 0);
        }
        if (pos > ClimberConstants.upperLimit) {
            speedClamped = Math.min(speed, 0);
        }
        return speedClamped;
    }
}
