package frc.robot;
import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.drive.Module.ModuleInfo;

public final class Constants {
    public static enum PivotId { FL, FR, BL, BR;}
    public static class LimelightConstants {
        public static final double limelightAngle = -10.0;
        public static final double limelightLensHeight = 11.0;
        public static final double limelightLensHorrizontalDisplacement = 0.0;
        public static final double horizontalResolution = 0.0;
        public static final double verticalResolution = 0.0;
        public static final double noteHeightInches = 2.0;
    }
    public static class VisionConstants{
        public static final double xyStdDev = 0.3;
        public static final double thetaStdDev = 0.3;
    }
    public static class SwerveDriveDimensions {
        public static final double wheelRadius = 0.053975;
        public static final double driveGearRatio = 7.73;
        public static final double steerGearRatio = 43.6;
        public static final double wheelYPos = Units.inchesToMeters(10.375);
        public static final double wheelXPos = Units.inchesToMeters(12.375);
        public static final double maxSpeed = 4;

        private static final Translation2d frontLeftLocation = new Translation2d(wheelXPos, wheelYPos);
        private static final Translation2d frontRightLocation = new Translation2d(wheelXPos, -wheelYPos);
        private static final Translation2d backLeftLocation = new Translation2d(-wheelXPos, wheelYPos);
        private static final Translation2d backRightLocation = new Translation2d(-wheelXPos, -wheelYPos);

        public static final Translation2d[] positions = new Translation2d[]{frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation};

        public static final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
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
            1, //5, 
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

    public static class IntakeConstants{
        public static final int intakeCanID = 6;
        public static final int indexerCanID = 16;
        public static final double proximityVoltageThreshold = 4.0;
        public static final int proximitySensorChannel = 0;
    }

    public static class ClimberConstants{
        public static final int leftCanID = 0;
        public static final int rightCanID = 0;
        public static final double lowerLimit = 0;
        public static final double upperLimit = 90;
    }

    public static class ShooterConstants{
        public static final int topLeftCanID = 21; 
        public static final int bottomLeftCanID = 6;
        public static final int topRightCanID = 13;
        public static final int bottomRightCanID = 14;
    }

    public static class PIDConstants{
        public static HashMap<String, PIDController> map = new HashMap<>();
        public static PIDController constructPID(PIDController controller, String name){
            PIDController n = new PIDController(controller.getP(), controller.getI(), controller.getD());
            map.put(name, n);
            return n;
        }
        

        //controllers
        public static PIDController rotPID = new PIDController(0.6, 0.00000, 0.000);
        public static PIDController rotMovingPID = new PIDController(1, 0, 0);
        public static PIDController gyroCorrectPid = new PIDController(1, 0, 0);
        public static PIDController driveForwardPID = new PIDController(0.5, 0, 0);
        public static PIDController targetingPID = new PIDController(0.1, 0, 0);
        public static PIDController horizontalMLVision = new PIDController(0.008, 0, 0);
        public static PIDController rotMLVision = new PIDController(0.004, 0, 0);
        public static PIDController extensionPID = new PIDController(0.03, 0.00, 0.00); // TODO values from sim: 3, 1, 0
        public static PIDController climberPID = new PIDController(0.01, 0, 0);
    }
    public static class FieldConstants{
        public static double height = 8.21;
        public static double width = 16.54;
        public static Translation2d ampPositionRed = new Translation2d(14.667, 7.4);
        public static Translation2d ampPositionBlue = new Translation2d(1.7, 7.4);
        public static Translation2d speakerPositionRed = new Translation2d(15.214 + Units.feetToMeters(3), 5.555);
        public static Translation2d speakerPositionBlue = new Translation2d(1.328 - Units.feetToMeters(3), 5.555);
    }

    public static class TargetingConstants {
        public static int leftTargetingMotorId = 12;
        public static int rightTargetingMotorId = 14; // TODO replace all of these constants
        public static double targetingLowerLimit = 0;
        public static double targetingUpperLimit = 90;
        public static double targetingManualSpeed = 0.5; // TODO speed
        
        public static double extensionLowerLimit = 0.0; // TODO limit
        public static double extensionUpperLimit = 1.0; // TODO limit

        public static double targetingMinVoltage = 0.05;
        public static double targetingMaxVoltage = 4.95;

        public static int extensionMotorId = 15;
        public static double extensionManualSpeed = 0.5;
    }
}
