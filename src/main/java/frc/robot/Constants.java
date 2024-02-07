package frc.robot;
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
        public static final double xyStdDev = 0.5;
        public static final double thetaStdDev = 0.5;
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
            5, 
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

    public static class ShooterConstants{
        public static final int topLeftCanID = 21; 
        public static final int bottomLeftCanID = 6;
        public static final int topRightCanID = 13;
        public static final int bottomRightCanID = 14;
    }

    public static class PIDConstants{
        public static PIDController constructPID(PIDController controller){
            return new PIDController(controller.getP(), controller.getI(), controller.getD());
        }

        //controllers
        public static PIDController rotPID = new PIDController(0.45, 0.00000, 0.00);
        public static PIDController driveForwardPID = new PIDController(0.8, 0, 0);
        public static PIDController targetingPID = new PIDController(0.1, 0, 0);
    }
    public static class FieldConstants{
        public static double height = 8.21;
        public static double width = 16.54;
        public static Translation2d ampPositionRed = new Translation2d(14.667, 7.8);
        public static Translation2d ampPositionBlue = new Translation2d(1.859, 7.803);
        public static Translation2d speakerPositionRed = new Translation2d(15.214, 5.555);
        public static Translation2d speakerPositionBlue = new Translation2d(1.328, 5.555);
    }

    public static class TargetingConstants {
        public static int leftTargetingMotorId = 12;
        public static int rightTargetingMotorId = 14; // TODO replace all of these constants
        public static double targetingLowerLimit = 0;
        public static double targetingUpperLimit = 90;
        public static double targetingManualSpeed = 0.5; // 
    }
}
