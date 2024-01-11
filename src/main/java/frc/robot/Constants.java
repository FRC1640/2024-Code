package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.ModuleInfo;

public final class Constants {
    public static enum PivotId { FL, FR, BL, BR;}
    public static class SwerveDriveDimensions {
        public static final double wheelRadius = 0.053975;
        public static final double driveGearRatio = 7.73;
        public static final double steerGearRatio = 0.02291666667;
        public static final double wheelYPos = Units.inchesToMeters(10.375);
        public static final double wheelXPos = Units.inchesToMeters(12.375);
        public static final double maxSpeed = 4;
    }
    public static class SimulationConstants{
        public static final double roomTempCelsius = 23;
        
    }
    public static class ModuleConstants{
        public static final double minVoltage = 0.05;
        public static final double maxVoltage = 4.95;
        
        public static final ModuleInfo FL = new ModuleInfo(
            PivotId.FL,
            3, 
            4, 
            0, 
            45, 
            true, 
            true,
            true);

        public static final ModuleInfo FR = new ModuleInfo(
            PivotId.FR, 
            2, 
            1, 
            2, 
            -45, 
            true, 
            true,
            true);

        public static final ModuleInfo BL = new ModuleInfo(
            PivotId.BL, 
            18, 
            17, 
            1, 
            135, 
            true, 
            true,
            true);

        public static final ModuleInfo BR = new ModuleInfo(
            PivotId.BR, 
            19, 
            20, 
            3, 
            -135, 
            true, 
            true,
            true);
    }

}
