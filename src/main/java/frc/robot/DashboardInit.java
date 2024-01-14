package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DashboardInit {
    public DashboardInit() {

    }

    public static void matchInit() {
        // testing the theory: Endgame boolean
        ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");
        teleop.addBoolean("Endgame", () -> DriverStation.getMatchTime() >= 130).withSize(1, 1).withPosition(5, 0);
        // Does this return in seconds/all of the match? online source says returns in
        // terms of auton vs teleop, better to check with below addition
        // BIG FEATURE 1: Big ol' match timer
        teleop.addDouble("Match Timer", () -> DriverStation.getMatchTime()).withSize(4, 3).withPosition(1, 0);
        // BIG FEATURE 2: Is the robot holding a note?
        // teleop.addBoolean("Note?"() -> [if sensor yes? - is there a sensor
        // here?]).withSize(5,5).withPosition(2, 0);
        // BIG FEATURE 3: camera feed
        // DON'T WORRY I WILL CHANGE THE TITLE I KNOW ITS DUMB ITS A PLACEHOLDER (so are
        // the camera name and url in case you couldn't tell)
        //teleop.addCamera("Robot Vision 👀", "camera name", "camera url").withSize(5, 5).withPosition(0, 1);

        // Justin suggesion: Limelight info? but like what info tho
    }

}
