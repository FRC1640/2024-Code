package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.drive.Module.ModuleInfo;

public final class Constants {
    public static enum PivotId {
        FL, FR, BL, BR;
    }

    public static class MLVisionLimelightConstants { // TODO: set these
        public static final double limelightAngle = -10.0;
        public static final double limelightLensHeight = 11.0;
        public static final double noteHeightInches = 2.0;
    }

    public static class AprilTagVisionConstants {
        public static final double xyStdDev = 2.5;
        public static final double thetaStdDev = 99999999;

        public static final double xyStdDevAuto = 2;
        public static final double thetaStdDevAuto = 99999999;
    }

    public static class SwerveDriveDimensions {
        
        public static final double driveGearRatio = 116/15;
        public static final double steerGearRatio = 43.6;
        public static final double wheelYPos = Units.inchesToMeters(22.75 / 2);
        public static final double wheelXPos = Units.inchesToMeters(22.75 / 2);
        public static final double maxSpeed = 4.267;

        private static final Translation2d frontLeftLocation = new Translation2d(wheelXPos, wheelYPos);
        private static final Translation2d frontRightLocation = new Translation2d(wheelXPos, -wheelYPos);
        public static final Translation2d backLeftLocation = new Translation2d(-wheelXPos, wheelYPos);
        public static final Translation2d backRightLocation = new Translation2d(-wheelXPos, -wheelYPos);

        public static final Translation2d[] positions = new Translation2d[] { frontLeftLocation, frontRightLocation,
                backLeftLocation, backRightLocation };

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    }

    public static class SimulationConstants {
        public static final double roomTempCelsius = 23;

    }

    public static class ModuleConstants {
        public static final double minVoltage = 0.05;
        public static final double maxVoltage = 4.95;

        public static final ModuleInfo FL = new ModuleInfo(
                PivotId.FL,
                3,
                2,
                0,
                45,
                true,
                true,
                true,
                Units.inchesToMeters(3.7432661290322 / 2));

        public static final ModuleInfo FR = new ModuleInfo(
                PivotId.FR,
                9, // 2023: and dew 1: 2
                8, // 2023: 1, dew 1: 5
                2,
                -45,
                true,
                true,
                true,
                Units.inchesToMeters(3.7432661290322 / 2));

        public static final ModuleInfo BL = new ModuleInfo(
                PivotId.BL,
                5,
                4,
                1,
                135,
                true,
                true,
                true,
                Units.inchesToMeters(3.7432661290322 / 2));

        public static final ModuleInfo BR = new ModuleInfo(
                PivotId.BR,
                7,
                6,
                3,
                -135,
                true,
                true,
                true,
                Units.inchesToMeters(3.7432661290322 / 2));
    }

    public static class IntakeConstants {
        public static final int intakeCanID = 14;
        public static final int indexerCanID = 19;
        public static final double proximityVoltageThreshold = 4.0;
        public static final int proximitySensorChannel = 9;
    }

    public static class ClimberConstants {
        public static final int leftCanID = 18;
        public static final int rightCanID = 17;
        public static final double lowerLimit = -15;
        public static final double upperLimit = 80;
        public static final int leftClimberResolver = 6;
        public static final int rightClimberResolver = 5;
        public static final int leftProximityChannel = 8;
        public static final int rightProximityChannel = 7;
    }

    public static class ShooterConstants {
        public static final int topLeftCanID = 13;
        public static final int bottomLeftCanID = 12;
        public static final int topRightCanID = 11;
        public static final int bottomRightCanID = 10;
        public static double waitTime = 1;
    }

    public static class PIDConstants {
        public static HashMap<String, PIDController> map = new HashMap<>();

        public static PIDController constructPID(PIDController controller, String name) {
            PIDController n = new PIDController(controller.getP(), controller.getI(), controller.getD());
            map.put(name, n);
            return n;
        }

        // controllers
        public static PIDController rotPID = new PIDController(0.4, 0.0001, 0.000);
        public static PIDController rotMovingPID = new PIDController(0.5, 0.0001, 0);
        public static PIDController gyroCorrectPid = new PIDController(0.1, 0, 0);
        public static PIDController driveForwardPID = new PIDController(0.5, 0, 0);
        public static PIDController targetingPID = new PIDController(0.25, 0, 0);
        public static PIDController horizontalMLVision = new PIDController(0.006, 0, 0);
        public static PIDController horizontalMLVisionDrive = new PIDController(0.008, 0, 0);
        public static PIDController rotMLVision = new PIDController(0.0045, 0, 0);
        
        public static PIDController extensionPID = new PIDController(0.5, 0.00, 0.00); // values from sim: 3, 1, 0
        public static PIDController climberPID = new PIDController(0.01, 0, 0);
        
        public static PIDController radianAngle = new PIDController(0.1, 0, 0);

        public static PIDController rotToSpeaker = new PIDController(0.001, 0.0001, 0.0001);

        public static PIDController shooterVelocityPID = new PIDController(0.01, 0.06, 0.00005);

        public static PIDController drivePIDController = new PIDController(0.49677, 0.0, 0);

        public static PIDController turningPIDController = new PIDController(0.725, 0.0, 0.005);

    }

    public static class FieldConstants {
        public static double height = 8.21;
        public static double width = 16.54;
        public static Translation2d ampPositionRed = new Translation2d(14.9, 7.840);
        public static Translation2d ampPositionBlue = new Translation2d(1.860, 7.840);
        public static Translation2d speakerPositionRed = new Translation2d(width - 0.4, 5.5);
        public static Translation2d speakerPositionBlue = new Translation2d(0.4, 5.5);
        public static Pose2d[] blueStages = {new Pose2d(new Translation2d(4.2, 5.1), new Rotation2d(2.1)), new Pose2d(new Translation2d(4.3, 3.1), new Rotation2d(-2.2)), new Pose2d(new Translation2d(6, 3.9), new Rotation2d(0))}; // 0: Amp/Speaker Chain (steve), 1: Source Facing Chain (Brenda), 2: the other one (kevin)
        public static Pose2d[] redStages = {new Pose2d(new Translation2d(12.3, 5), new Rotation2d(1)), new Pose2d(new Translation2d(12.3,3.2), new Rotation2d(-1)), new Pose2d(new Translation2d(10.5,4.0), new Rotation2d(3.14159265))};
        
        public static Translation2d stashPositionBlue = new Translation2d(1.43, 5.5);
        public static Translation2d stashPositionRed = new Translation2d(14.643, 5.5);
        
        public static double fullCourtShootingRadius = 7; // 6.5 = x blue line, 10.7 = x red line 10.24
 
    }

    public static class TargetingConstants {
        public static int leftAngleMotorId = 200;
        public static int rightAngleMotorId = 15;
        public static double angleLowerLimit = 29;
        public static double angleUpperLimit = 110;
        public static double angleManualSpeed = 0.05;
        public static double angleError = 0.5;

        public static double extensionLowerLimit = 5;
        public static double extensionUpperLimit = 100;
        public static double extensionUpperLimitTrap = 100;



        public static double angleMinVoltage = 0.05;
        public static double angleMaxVoltage = 4.95;

        public static int extensionMotorId = 16;
        public static double extensionManualSpeed = 0.85;

        public static int resolverID = 4;
    }

    public static void updateStatusFrames(CANSparkMax motor, int status0, int status1, int status2,
            int status3, int status4, int status5, int status6) {

        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, status3);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, status4);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, status5);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, status6);

    }
}
