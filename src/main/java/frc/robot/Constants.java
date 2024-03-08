package frc.robot;
import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.drive.Module.ModuleInfo;

public final class Constants {
    public static enum PivotId { FL, FR, BL, BR;}
    public static class MLVisionLimelightConstants { // TODO: set these
        public static final double limelightAngle = -10.0;
        public static final double limelightLensHeight = 11.0;
        public static final double noteHeightInches = 2.0;
    }
    public static class AprilTagVisionConstants{
        public static final double xyStdDev = 0.25;
        public static final double thetaStdDev = 1;

        public static final double xyStdDevAuto = 0.25;
        public static final double thetaStdDevAuto = 99999999;
    }
    public static class SwerveDriveDimensions {
        public static final double wheelRadius = Units.inchesToMeters(4.23/2);
        public static final double driveGearRatio = 7.73;
        public static final double steerGearRatio = 43.6;
        public static final double wheelYPos = Units.inchesToMeters(22.75/2);
        public static final double wheelXPos = Units.inchesToMeters(22.75/2);
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
            2, 
            21, 
            0, 
            45, 
            true, 
            true,
            true);

        public static final ModuleInfo FR = new ModuleInfo(
            PivotId.FR, 
            19, // 2023: and dew 1: 2
            20, //2023: 1, dew 1: 5
            2,
            -45, 
            true, 
            true,
            true);

        public static final ModuleInfo BL = new ModuleInfo(
            PivotId.BL, 
            9, 
            10, 
            1, 
            135, 
            true, 
            true,
            true);

        public static final ModuleInfo BR = new ModuleInfo(
            PivotId.BR, 
            12, 
            11, 
            3, 
            -135, 
            true, 
            true,
            true);
    }

    public static class IntakeConstants{
        public static final int intakeCanID = 15;
        public static final int indexerCanID = 4;
        public static final double proximityVoltageThreshold = 4.0;
        public static final int proximitySensorChannel = 9;
    }

    public static class ClimberConstants{
        public static final int leftCanID = 17;
        public static final int rightCanID = 16;
        public static final double lowerLimit = -15;
        public static final double upperLimit = 80;
        public static final int leftClimberResolver = 6;
        public static final int rightClimberResolver = 7;
    }

    public static class ShooterConstants{
        public static final int topLeftCanID = 8; 
        public static final int bottomLeftCanID = 7;
        public static final int topRightCanID = 6;
        public static final int bottomRightCanID = 5;
        public static double waitTime = 1;
    }

    public static class PIDConstants{
        public static HashMap<String, PIDController> map = new HashMap<>();
        public static PIDController constructPID(PIDController controller, String name){
            PIDController n = new PIDController(controller.getP(), controller.getI(), controller.getD());
            map.put(name, n);
            return n;
        }
        //controllers
        public static PIDController rotPID = new PIDController(0.4, 0.00000, 0.000);
        public static PIDController rotMovingPID = new PIDController(0.4, 0, 0);
        public static PIDController gyroCorrectPid = new PIDController(0.1, 0, 0);
        public static PIDController driveForwardPID = new PIDController(0.5, 0, 0);
        public static PIDController targetingPID = new PIDController(0.01, 0, 0);
        public static PIDController horizontalMLVision = new PIDController(0.007, 0, 0);
        public static PIDController rotMLVision = new PIDController(0.006, 0, 0);
        public static PIDController extensionPID = new PIDController(0.03, 0.00, 0.00); // values from sim: 3, 1, 0
        public static PIDController climberPID = new PIDController(0.01, 0, 0);
        public static PIDController radianAngle = new PIDController(0.1, 0, 0);

        public static PIDController rotToSpeaker = new PIDController(0.001, 0.0001, 0.0001);

        
        public static PIDController shooterVelocityPID = new PIDController(0.01, 0.06, 0.00005);
        

        public static PIDController drivePIDController = new PIDController(0.49677, 0.0, 0);

        public static PIDController turningPIDController = new PIDController(0.725, 0.0, 0.005); 
        
    }
    public static class FieldConstants{
        public static double height = 8.21;
        public static double width = 16.54;
        public static Translation2d ampPositionRed = new Translation2d(14.701, 7.881);
        public static Translation2d ampPositionBlue = new Translation2d(2.064, 7.8);
        public static Translation2d speakerPositionRed = new Translation2d(16.789, 5.590);
        public static Translation2d speakerPositionBlue = new Translation2d(0.4, 5.544);
    }

    public static class TargetingConstants {
        public static int leftAngleMotorId = 14;
        public static int rightAngleMotorId = 13;
        public static double angleLowerLimit = 29;
        public static double angleUpperLimit = 90;
        public static double angleManualSpeed = 0.05;
        public static double angleError =2;
        
        public static double extensionLowerLimit = 0;
        public static double extensionUpperLimit = 100.0;

        public static double angleMinVoltage = 0.05;
        public static double angleMaxVoltage = 4.95;

        public static int extensionMotorId = 3;
        public static double extensionManualSpeed = 0.5;

        public static int resolverID = 4;
    }
}
