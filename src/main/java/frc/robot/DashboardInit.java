package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot.TestMode;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Writes various pieces of match data to Shuffleboard.
 */
public class DashboardInit {
    private static SendableChooser<Command> autoChooser;
    private static SendableChooser<TestMode> testModeChooser = new SendableChooser<TestMode>();
    private static SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
    public DashboardInit() {

    }
    //inits all of shuffleboard
    public static void init(DriveSubsystem driveSubsystem, CommandXboxController controller){
        autonInit();
        matchInit();
        testInit();
        sysidInit(driveSubsystem,controller);
    }

    private static void autonInit() {
        // add pathplanner autochooser
        autoChooser = AutoBuilder.buildAutoChooser();
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
        autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);
    }
    private static void testInit(){
        ShuffleboardTab testTab = Shuffleboard.getTab("Test Chooser");
        for (int i = 0; i < TestMode.values().length; i++){
            testModeChooser.addOption(TestMode.values()[i].toString(), TestMode.values()[i]);
        }
        testTab.add(testModeChooser).withSize(5, 5).withPosition(1, 1);
    }
    private static void matchInit() {
        // ENDGAME INDICATOR (Number is 21 because shuffleboard is pretending the = in
        // <= doesn't exist)
        ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");
        teleop.addBoolean("Endgame", () -> DriverStation.getMatchTime() <= 21 && DriverStation.isTeleop())
                .withSize(3, 3).withPosition(0, 1);
        // BIG FEATURE 1: Big ol' match timer
        teleop.addDouble("Match Timer", () -> Math.round(DriverStation.getMatchTime() * 10000) / 10000).withSize(2, 1)
                .withPosition(0, 0);
    }
    private static void sysidInit(DriveSubsystem driveSubsystem, CommandXboxController controller){//TODO sequential command where each command ends on button press
        ShuffleboardTab sysidTab = Shuffleboard.getTab("Sysid");
        sysidChooser.addOption("SwerveSysID", new SequentialCommandGroup(driveSubsystem.sysIdQuasistatic(null)));
        sysidTab.add(sysidChooser).withSize(5, 5).withPosition(1, 1);
    } 
    public static TestMode getTestMode(){
        return testModeChooser.getSelected();
    }
    public static Command getAutoChooserCommand(){
        return autoChooser.getSelected();
    }

}
