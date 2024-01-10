package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class SwerveDriveDimensions {
        public static final double wheelRadius = 0.053975;
        public static final double driveGearRatio = 7.73;
        public static final double steerGearRatio = 1;
        public static final double wheelYPos = Units.inchesToMeters(10.375);
        public static final double wheelXPos = Units.inchesToMeters(12.375);
        public static final double maxSpeed = 4;
    }
    public static class SimulationConstants{
        public static final double roomTempCelsius = 23;
    }

}
