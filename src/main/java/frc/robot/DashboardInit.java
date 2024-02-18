package frc.robot;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import javax.management.openmbean.TabularType;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.drive.DriveSubsystem;
import frc.lib.sysid.CreateSysidCommand;
import frc.robot.Robot.TestMode;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVision;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;

/**
 * Writes various pieces of match data to Shuffleboard.
 */
public class DashboardInit {
    private static SendableChooser<Command> autoChooser;
    private static SendableChooser<TestMode> testModeChooser = new SendableChooser<TestMode>();
    private static SendableChooser<Command> sysidChooser = new SendableChooser<Command>();

    private static DriveSubsystem driveSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static ClimberSubsystem climberSubsystem;
    private static ShooterSubsystem shooterSubsystem;
    private static TargetingSubsystem targetingSubsystem;
    private static CommandXboxController controller;

    private static final Field2d field = new Field2d();
    public DashboardInit() {

    }

    // inits all of shuffleboard
    public static void init(DriveSubsystem driveSubsystem, CommandXboxController controller,
            AprilTagVision vision, IntakeSubsystem intakeSubsystem, ClimberSubsystem climberSubsystem,
                ShooterSubsystem shooterSubsystem, TargetingSubsystem targetingSubsystem) {
        autonInit();
        matchInit(vision);
        testInit();
        DashboardInit.driveSubsystem = driveSubsystem;
        DashboardInit.controller = controller;
        DashboardInit.intakeSubsystem = intakeSubsystem;
        DashboardInit.climberSubsystem = climberSubsystem;
        DashboardInit.shooterSubsystem = shooterSubsystem;
        DashboardInit.targetingSubsystem = targetingSubsystem;
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

    private static void onTestChange(TestMode mode){
        switch (mode) {
            case SYSID:
                sysidInit(driveSubsystem, controller);
                break;

            case MOTOR:
                motorInit(intakeSubsystem, climberSubsystem, shooterSubsystem, targetingSubsystem);
                break;
        
            default:
                break;
        }
    }

    private static void matchInit(AprilTagVision vision) { // TODO Limelight feed appears only when Shuffleboard is running before sim starts. Why?
        // ENDGAME INDICATOR
        ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");
        teleop.addBoolean("Endgame", () -> DriverStation.getMatchTime() <= 21 && DriverStation.isTeleop())
                .withSize(1, 3)
                .withPosition(0, 1);
        // MATCH TIMER
        teleop.addDouble("Match Timer", () -> Math.round(DriverStation.getMatchTime() * 10000) / 10000)
                .withSize(1, 1)
                .withPosition(0, 0);
        // LIMELIGHT STREAM?
        teleop.addCamera("Limelight Feed", "limelight camera(placeholder?)", "http://10.16.40.109:5800/stream.mjpg")
                .withSize(4,4)
                .withPosition(1,0);
        teleop.addBoolean("Apriltag Sighted?", () -> vision.isTarget())
                .withSize(1, 2)
                .withPosition(5, 2);
        teleop.add(field)
            .withSize(4, 4)
            .withPosition(6, 0);
    }

    public static void setFieldPos(Pose2d pose){
        field.setRobotPose(pose);
    }

    private static void sysidInit(DriveSubsystem driveSubsystem, CommandXboxController controller) {
        ShuffleboardTab sysidTab = Shuffleboard.getTab("Sysid");
        sysidChooser.setDefaultOption("None!", new WaitCommand(0.1));
        sysidChooser.addOption("SwerveSysID",
                CreateSysidCommand.createCommand(driveSubsystem::sysIdQuasistatic, driveSubsystem::sysIdDynamic,
                        "SwerveSysId", ()->controller.a().getAsBoolean(), ()->controller.b().getAsBoolean()));
        sysidTab.add(sysidChooser).withSize(5, 5).withPosition(1, 1);
    }

    private static void motorInit(IntakeSubsystem intakeSubsystem, ClimberSubsystem climberSubsystem,
            ShooterSubsystem shooterSubsystem, TargetingSubsystem targetingSubsystem) { // TODO motor ids
        DoubleConsumer[] consumers = {(intakeSpeed) -> intakeSubsystem.testIntakeSpeedCommand(intakeSpeed),
            (indexerSpeed) -> intakeSubsystem.testIndexerSpeedCommand(indexerSpeed),
            (climberSpeed) -> climberSubsystem.setSpeedCommand(climberSpeed, climberSpeed),
            (topLeftSpeed) -> shooterSubsystem.testTopLeftSpeed(topLeftSpeed),
            (topRightSpeed) -> shooterSubsystem.testTopRightSpeed(topRightSpeed),
            (bottomLeftSpeed) -> shooterSubsystem.testBottomLeftSpeed(bottomLeftSpeed),
            (bottomRightSpeed) -> shooterSubsystem.testBottomRightSpeed(bottomRightSpeed),
            (angleSpeed) -> targetingSubsystem.setAnglePercentOutputCommand(angleSpeed),
            (extensionSpeed) -> targetingSubsystem.setExtensionPercentOutputCommand(extensionSpeed)};
        ArrayList<DoubleConsumer> motorSetSpeed = new ArrayList<>(Arrays.asList(consumers));
        DoubleSupplier[] suppliers = {() -> intakeSubsystem.getIntakePercentOutput(),
            () -> intakeSubsystem.getIndexerPercentOutput(), () -> climberSubsystem.getPercentOutput(),
            () -> shooterSubsystem.getTopLeftSpeed(), () -> shooterSubsystem.getTopRightSpeed(),
            () -> shooterSubsystem.getBottomLeftSpeed(), () -> shooterSubsystem.getBottomRightSpeed()};
        ShuffleboardTab motorTab = Shuffleboard.getTab("Motors");
        
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
