package frc.robot;

import java.util.ArrayList;
import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.drive.DriveSubsystem;
import frc.lib.sysid.CreateSysidCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TargetingConstants;
import frc.robot.Robot.TestMode;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVision;
import frc.robot.sensors.Vision.MLVision.MLVision;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.util.dashboard.PIDUpdate;

/**
 * Writes various pieces of match data to Shuffleboard.
 */
public class DashboardInit {
    private static SendableChooser<Command> autoChooser;
    private static SendableChooser<TestMode> testModeChooser = new SendableChooser<TestMode>();
    private static SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
    private static SendableChooser<PIDController> pidChooser = new SendableChooser<PIDController>(); // what put here ??

    private static DriveSubsystem driveSubsystem;
    private static XboxController controller;
    private static ShooterSubsystem shooterSubsystem;
    private static TargetingSubsystem targetingSubsystem;
    private static MLVision mlVision;

    private static final Field2d field = new Field2d();

    static boolean sysIdInit = false;
    static boolean pidInit = false;

    private static GenericEntry kP;
    private static GenericEntry kI;
    private static GenericEntry kD;
    private static GenericEntry kS;

    public DashboardInit() {

    }

    // inits all of shuffleboard
    public static void init(DriveSubsystem driveSubsystem, XboxController controller, ArrayList<AprilTagVision> vision, TargetingSubsystem targetingSubsystem, ShooterSubsystem shooterSubsystem, MLVision mlVision) {
        DashboardInit.mlVision = mlVision;
        DashboardInit.driveSubsystem = driveSubsystem;
        DashboardInit.targetingSubsystem = targetingSubsystem;
        DashboardInit.controller = controller;
        DashboardInit.shooterSubsystem = shooterSubsystem;
        autonInit();
        matchInit(vision);
        testInit();

    }

    private static void autonInit() {
        // add pathplanner autochooser
        autoChooser = AutoBuilder.buildAutoChooser();
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
        autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);
    }

    private static void testInit() {
        ShuffleboardTab testTab = Shuffleboard.getTab("Test Chooser");
        testModeChooser.setDefaultOption("NONE", TestMode.NONE);
        for (int i = 1; i < TestMode.values().length; i++) {
            System.out.println(TestMode.values()[i].toString());
            testModeChooser.addOption(TestMode.values()[i].toString(), TestMode.values()[i]);
        }
        testTab.add(testModeChooser).withSize(5, 5).withPosition(1, 1);
        testModeChooser.onChange(DashboardInit::onTestChange);
    }

    private static void pidInit() {
        ShuffleboardTab PIDTab = Shuffleboard.getTab("PID");
        pidChooser.setDefaultOption("None", new PIDController(0, 0, 0));
        for (Map.Entry<String, PIDController> entry : PIDConstants.map.entrySet()){
            pidChooser.addOption(entry.getKey(), entry.getValue());
        }
        PIDTab.add(pidChooser)
                .withSize(4, 4)
                .withPosition(0, 1);
        
        kP = PIDTab.add("P", 0).withSize(1,1).withPosition(0,0).getEntry();
        kI = PIDTab.add("I", 0).withSize(1,1).withPosition(1,0).getEntry();
        kD = PIDTab.add("D", 0).withSize(1,1).withPosition(2,0).getEntry();
        kS = PIDTab.add("Setpoint", 0).withSize(1,1).withPosition(3,0).getEntry();
        PIDUpdate.setEntries(kP, kI, kD, kS);
        pidChooser.onChange(DashboardInit::onPIDChooserChange);
        pidInit = true;
    }

    private static void onPIDChooserChange(PIDController controller) {
        PIDUpdate.setPID(controller);
        kP.setDouble(controller.getP());
        kI.setDouble(controller.getI());
        kD.setDouble(controller.getD());
    }

    private static void onTestChange(TestMode mode) {
        switch (mode) {
            case SYSID:
                if (!sysIdInit){
                    sysidInit();
                }
                break;

            case PID:
                if (!pidInit){
                    pidInit();
                }
                break;

            default:
                break;
        }
    }

    private static void matchInit(ArrayList<AprilTagVision> vision) {
        // ENDGAME INDICATOR
        BooleanSupplier visionOne = () -> vision.get(0).isTarget();
        BooleanSupplier visionTwo = () -> vision.get(1).isTarget();
        ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");
        teleop.addBoolean("Endgame", () -> DriverStation.getMatchTime() <= 21 && DriverStation.isTeleop())
                .withSize(1, 3)
                .withPosition(0, 1);
        // MATCH TIMER
        teleop.addDouble("Match Timer", () -> Math.round(DriverStation.getMatchTime() * 10000) / 10000)
                .withSize(1, 1)
                .withPosition(0, 0);
        // LIMELIGHT STREAM
        teleop.addCamera("Limelight Feed", "limelight camera(placeholder?)", "http://10.16.40.219:5800/stream.mjpg")
                .withSize(4, 4)
                .withPosition(1, 0);
        // teleop.addBoolean("Apriltag Sighted?", () -> vision.isTarget())
        //         .withSize(1, 2)
        //         .withPosition(5, 2);
        teleop.addBoolean("Apriltag 1", visionOne)
                .withSize(1,1)
                .withPosition(5,2);
        teleop.addBoolean("Apriltag 2", visionTwo)
                .withSize(1,1)
                .withPosition(5,3);
        teleop.addBoolean("Note sighted?", () -> mlVision.isTarget())
            .withSize(1, 2)
            .withPosition(5, 0);
        teleop.add(field)
                .withSize(4, 2)
                .withPosition(6, 0);
        teleop.addBoolean("Is targeting right?", () -> targetingSubsystem.isAnglePositionAccurate(TargetingConstants.angleError))
                .withSize(1,1)
                .withPosition(7,2);
        teleop.addBoolean("Targeting at limit?", () -> posGet())
            .withSize(1,1)
            .withPosition(8,2);
        teleop.addBoolean("Is rotation right?", () -> driveSubsystem.getRotAccuracy()) 
            .withSize(1,1)
            .withPosition(8,3);
    }

    public static boolean posGet(){
            return (TargetingConstants.angleLowerLimit >= Math.toDegrees(targetingSubsystem.getAnglePosition()) || Math.toDegrees((targetingSubsystem.getAnglePosition())) >= TargetingConstants.angleUpperLimit);
        }


    public static void setFieldPos(Pose2d pose) {
        field.setRobotPose(pose);
    }

    private static void sysidInit() {
        ShuffleboardTab sysidTab = Shuffleboard.getTab("Sysid");
        sysidChooser.setDefaultOption("None!", new WaitCommand(0.1));
        sysidChooser.addOption("AnglerSysID", CreateSysidCommand.createCommand(targetingSubsystem::sysIdQuasistatic, 
        targetingSubsystem::sysIdDynamic, "AnglerSysID", () -> controller.getAButton(), () -> controller.getBButton()));
        sysidChooser.addOption("SwerveSysID",
                CreateSysidCommand.createCommand(driveSubsystem::sysIdQuasistatic, driveSubsystem::sysIdDynamic,
                        "SwerveSysId", () -> controller.getAButton(), () -> controller.getBButton()));

        sysidChooser.addOption("ShooterSysID",
            CreateSysidCommand.createCommand(
                shooterSubsystem::sysIdQuasistatic, 
                shooterSubsystem::sysIdDynamic, 
                "ShooterSysID", () -> controller.getAButton(), () -> controller.getBButton()));
        sysidTab.add(sysidChooser).withSize(5, 5).withPosition(1, 1);
        sysIdInit = true;
    }

    public static TestMode getTestMode() {
        return testModeChooser.getSelected();
    }

    public static Command getSelectedSysid() {
        return sysidChooser.getSelected();
    }

    public static Command getAutoChooserCommand() {
        return autoChooser.getSelected();
    }
}
