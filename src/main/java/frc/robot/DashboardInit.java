package frc.robot;

import java.util.Map;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
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
import frc.robot.Constants.PIDConstants;
import frc.robot.Robot.TestMode;
import frc.robot.sensors.Vision.AprilTagVision.AprilTagVision;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.targeting.TargetingSubsystem;
import frc.robot.util.dashboard.PIDUpdate;
import frc.robot.util.dashboard.AltMotorUpdate;
import frc.robot.util.dashboard.DitriggerCommandSarcophagus;
import frc.robot.util.dashboard.MonotriggerCommandSarcophagus;
import frc.robot.util.dashboard.MotorUpdatePeriodicHandler;

/**
 * Writes various pieces of match data to Shuffleboard.
 */
public class DashboardInit {
    private static SendableChooser<Command> autoChooser;
    private static SendableChooser<TestMode> testModeChooser = new SendableChooser<TestMode>();
    private static SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
    private static SendableChooser<PIDController> pidChooser = new SendableChooser<PIDController>(); // what put here ??

    private static DriveSubsystem driveSubsystem;
    private static IntakeSubsystem intakeSubsystem;
    private static ClimberSubsystem climberSubsystem;
    private static ShooterSubsystem shooterSubsystem;
    private static TargetingSubsystem targetingSubsystem;
    private static CommandXboxController controller;

    private static final Field2d field = new Field2d();

    private static final int NUMBER_OF_MOTORS = 19;

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

    private static void pidInit() {
        ShuffleboardTab PIDTab = Shuffleboard.getTab("PID");
        pidChooser.setDefaultOption("None", new PIDController(0, 0, 0));
        for (Map.Entry<String, PIDController> entry : PIDConstants.map.entrySet()){
            pidChooser.addOption(entry.getKey(), entry.getValue());
        }
        PIDTab.add(pidChooser)
                .withSize(4, 4)
                .withPosition(0, 1);
        GenericEntry kP = PIDTab.add("P", 0).withSize(1,1).withPosition(0,0).getEntry();
        GenericEntry kI = PIDTab.add("I", 0).withSize(1,1).withPosition(1,0).getEntry();
        GenericEntry kD = PIDTab.add("D", 0).withSize(1,1).withPosition(2,0).getEntry();
        PIDUpdate.setEntries(kP, kI, kD);
        pidChooser.onChange(DashboardInit::onPIDChooserChange);
    }

    private static void onPIDChooserChange(PIDController controller) {
        PIDUpdate.setPID(controller);
    }

    private static void onTestChange(TestMode mode) {
        switch (mode) {
            case SYSID:
                sysidInit(driveSubsystem, controller);
                break;

            case MOTOR:
                motorInit(intakeSubsystem, climberSubsystem, shooterSubsystem, targetingSubsystem, driveSubsystem);
                break;

            case PID:
                pidInit();
                break;

            default:
                break;
        }
    }

    private static void matchInit(AprilTagVision vision) {
        // ENDGAME INDICATOR
        ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");
        teleop.addBoolean("Endgame", () -> DriverStation.getMatchTime() <= 21 && DriverStation.isTeleop())
                .withSize(1, 3)
                .withPosition(0, 1);
        // MATCH TIMER
        teleop.addDouble("Match Timer", () -> Math.round(DriverStation.getMatchTime() * 10000) / 10000)
                .withSize(1, 1)
                .withPosition(0, 0);
        // LIMELIGHT STREAM
        teleop.addCamera("Limelight Feed", "limelight camera(placeholder?)", "http://10.16.40.109:5800/stream.mjpg")
                .withSize(4, 4)
                .withPosition(1, 0);
        teleop.addBoolean("Apriltag Sighted?", () -> vision.isTarget())
                .withSize(1, 2)
                .withPosition(5, 2);
        teleop.add(field)
                .withSize(4, 4)
                .withPosition(6, 0);
    }

    public static void setFieldPos(Pose2d pose) {
        field.setRobotPose(pose);
    }

    private static void sysidInit(DriveSubsystem driveSubsystem, CommandXboxController controller) {
        ShuffleboardTab sysidTab = Shuffleboard.getTab("Sysid");
        sysidChooser.setDefaultOption("None!", new WaitCommand(0.1));
        sysidChooser.addOption("SwerveSysID",
                CreateSysidCommand.createCommand(driveSubsystem::sysIdQuasistatic, driveSubsystem::sysIdDynamic,
                        "SwerveSysId", () -> controller.a().getAsBoolean(), () -> controller.b().getAsBoolean()));
        sysidTab.add(sysidChooser).withSize(5, 5).withPosition(1, 1);
    }

    private static void motorInit(IntakeSubsystem intakeSubsystem, ClimberSubsystem climberSubsystem,
            ShooterSubsystem shooterSubsystem, TargetingSubsystem targetingSubsystem, DriveSubsystem driveSubsystem) { // TODO motor ids
        List<Function<DoubleSupplier, Command>> motorSetSpeed = new ArrayList<>();
        motorSetSpeed.add((speed) -> intakeSubsystem.testIntakeSpeedCommand(speed));
        motorSetSpeed.add((speed) -> intakeSubsystem.testIndexerSpeedCommand(speed));
        motorSetSpeed.add((speed) -> shooterSubsystem.testTopLeftSpeed(speed));
        motorSetSpeed.add((speed) -> shooterSubsystem.testTopRightSpeed(speed));
        motorSetSpeed.add((speed) -> shooterSubsystem.testBottomLeftSpeed(speed));
        motorSetSpeed.add((speed) -> shooterSubsystem.testBottomRightSpeed(speed));
        motorSetSpeed.add((speed) -> driveSubsystem.testFrontLeftVelocityCommand(speed));
        motorSetSpeed.add((speed) -> driveSubsystem.testFrontRightVelocityCommand(speed));
        motorSetSpeed.add((speed) -> driveSubsystem.testBackLeftVelocityCommand(speed));
        motorSetSpeed.add((speed) -> driveSubsystem.testBackRightVelocityCommand(speed));
        motorSetSpeed.add((angle) -> driveSubsystem.testFrontLeftAngleCommand(angle));
        motorSetSpeed.add((angle) -> driveSubsystem.testFrontRightAngleCommand(angle));
        motorSetSpeed.add((angle) -> driveSubsystem.testBackLeftAngleCommand(angle));
        motorSetSpeed.add((angle) -> driveSubsystem.testBackRightAngleCommand(angle));
        List<BiFunction<DoubleSupplier, BooleanSupplier, Command>> motorSetSpeedLimits = new ArrayList<>();
        motorSetSpeedLimits.add((speed, limits) -> climberSubsystem.updateClimberShuffleboardCommand(speed, limits));
        motorSetSpeedLimits.add((speed, limits) -> targetingSubsystem.updateAnglerShuffleboardCommand(speed, limits));
        motorSetSpeedLimits.add((speed, limits) -> targetingSubsystem.updateExtensionShuffleboardCommand(speed, limits));
        DoubleSupplier[] suppliers = {
            () -> intakeSubsystem.getIntakePercentOutput(),
            () -> intakeSubsystem.getIndexerPercentOutput(),
            () -> shooterSubsystem.getTopLeftSpeed(),
            () -> shooterSubsystem.getTopRightSpeed(),
            () -> shooterSubsystem.getBottomLeftSpeed(),
            () -> shooterSubsystem.getBottomRightSpeed(),
            () -> driveSubsystem.getFrontLeftVelocity(),
            () -> driveSubsystem.getFrontRightVelocity(),
            () -> driveSubsystem.getBackLeftVelocity(),
            () -> driveSubsystem.getBackRightVelocity(),
            () -> driveSubsystem.getFrontLeftAngle(),
            () -> driveSubsystem.getFrontRightAngle(),
            () -> driveSubsystem.getBackLeftAngle(),
            () -> driveSubsystem.getBackRightAngle(),
            () -> climberSubsystem.getPercentOutput(),
            () -> targetingSubsystem.getAnglerSpeedPercent(),
            () -> targetingSubsystem.getExtensionSpeedPercent()
        };
        List<DoubleSupplier> motorGetSpeed = new ArrayList<>(Arrays.asList(suppliers));
        String[] names = { "Intake", "Indexer", "Shooter TL", "Shooter TR", "Shooter BL",
            "Shooter BR", "Drive FL", "Drive FR", "Drive BL", "Drive BR", "Drive FL",
            "Drive FR", "Drive BL", "Drive BR", "Climber Motors", "Angler Motors",
            "Extension" };
        int[] coords = { 0, 4, 2, 4, 5, 4, 7, 4, 9, 4, 11, 4, 6, 0, 8, 0, 10, 0, 12, 0, 6, 2, 8, 2, 10, 2, 12, 2, 4, 0, 0, 0, 2, 0 };
        List<GenericEntry> sliderEntries = new ArrayList<>(NUMBER_OF_MOTORS);
        ShuffleboardTab motorTab = Shuffleboard.getTab("Motor");
        for (int i = 0; i < 6; i++) {
            sliderEntries.add(motorTab.add(names[i], 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -1, "max", 1, "display value", true))
                .withPosition(coords[i * 2], coords[i * 2 + 1])
                .withSize(2, 1)
                .getEntry());
            motorTab.addDouble(names[i] + " -Applied", motorGetSpeed.get(i))
                    .withWidget(BuiltInWidgets.kNumberBar)
                    .withProperties(Map.of("min", -1, "max", 1, "show text", true))
                    .withPosition(coords[i * 2], coords[i * 2 + 1] + 1)
                    .withSize(2, 1);
                // (i % 7) * 2, (i / 7) * 2 + 1
        }
        for (int i = 6; i < 10; i++) {
            sliderEntries.add(motorTab.add(names[i], 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -1, "max", 1, "display value", true))
                .withPosition(coords[i * 2], coords[i * 2 + 1])
                .withSize(2, 1)
                .getEntry());
            motorTab.addDouble(names[i] + " Velocity", motorGetSpeed.get(i))
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", -1, "max", 1, "show text", true))
                .withPosition(coords[i * 2], coords[i * 2 + 1] + 1)
                .withSize(2, 1);
        }
        for (int i = 10; i < 14; i++) {
            sliderEntries.add(motorTab.add(names[i] + "°", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -360, "max", 360, "show value", true))
                .withPosition(coords[i * 2], coords[i * 2 + 1])
                .withSize(2, 1)
                .getEntry());
        }
        for (int i = 14; i < 17; i++) {
            sliderEntries.add(motorTab.add(names[i], 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -1, "max", 1, "display value", true))
                .withPosition(coords[i * 2], coords[i * 2 + 1])
                .withSize(2, 1)
                .getEntry());
            motorTab.add(names[i] + " -Applied", 0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", -1, "max", 1, "show value", true))
                .withPosition(coords[i * 2], coords[i * 2 + 1] + 1)
                .withSize(2, 1);
        }
        motorTab.addDouble("Climber Encoder", () -> climberSubsystem.getEncoderValue()).withPosition(4, 2).withSize(1, 1);
        motorTab.addDouble("Angler Encoder", () -> targetingSubsystem.getAnglerEncoderValue()).withPosition(0, 2).withSize(1, 1);
        motorTab.addDouble("Extension Encoder", () -> targetingSubsystem.getExtensionEncoderValue()).withPosition(2, 2).withSize(1, 1);
        List<GenericEntry> toggliers = new ArrayList<>(3);
        toggliers.add(motorTab.add("Climber Limits", !climberSubsystem.getLimitsOff())
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(5, 2)
            .withSize(1, 1)
            .getEntry());
        toggliers.add(motorTab.add("Angler Limits", !targetingSubsystem.getAnglerLimitsOff())
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(1, 2)
            .withSize(1, 1)
            .getEntry());
        toggliers.add(motorTab.add("Extension Limits", !targetingSubsystem.getExtensionLimitsOff())
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(3, 2)
            .withSize(1, 1)
            .getEntry());
        List<AltMotorUpdate> updatiers = new ArrayList<>(19);
        for (int i = 0; i < 14; i++) {
            updatiers.add(new AltMotorUpdate(sliderEntries.get(i), motorSetSpeed.get(i)));
        }
        for (int i = 14; i < 17; i++) {
            updatiers.add(new AltMotorUpdate(sliderEntries.get(i), toggliers.get(i - 14), motorSetSpeedLimits.get(i - 14)));
        }
        MotorUpdatePeriodicHandler.giveMotorUpdates(updatiers.get(0), updatiers.get(1), updatiers.get(2),
            updatiers.get(3), updatiers.get(4), updatiers.get(5), updatiers.get(6), updatiers.get(7),
            updatiers.get(8), updatiers.get(9), updatiers.get(10), updatiers.get(11), updatiers.get(12),
            updatiers.get(13), updatiers.get(14), updatiers.get(15), updatiers.get(16));
        List<MonotriggerCommandSarcophagus> monoSarcophagi = new ArrayList<>(17);
        for (int i = 0; i < 14; i++) {
            monoSarcophagi.add(new MonotriggerCommandSarcophagus(updatiers.get(i).publishCommand(),
                sliderEntries.get(i), motorSetSpeed.get(i)));
        }
        List<DitriggerCommandSarcophagus> diSarcophagi = new ArrayList<>();
        for (int i = 14; i < 17; i++) {
            diSarcophagi.add(new DitriggerCommandSarcophagus(updatiers.get(i).publishCommand(),
                sliderEntries.get(i), toggliers.get(i - 14), motorSetSpeedLimits.get(i - 14)));
        }
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
