package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//ADD POSITION AND SIZE INFO FOR EVERYTHING
public class DashboardInit {
    public DashboardInit(){
    //testing the theory: Endgame boolean
        ShuffleboardTab teleop = Shuffleboard.getTab("Teleop");
        teleop.addBoolean("Endgame",() -> DriverStation.getMatchTime() >= 130);
        //Does this return in seconds/all of the match? online source says returns in terms of auton vs teleop, better to check with below addition
    //BIG FEATURE 1: Big ol' match timer
        teleop.addDouble("Match Timer", () -> DriverStation.getMatchTime());
    //BIG FEATURE 2: Is the robot holding a note?
        //teleop.addBoolean("Note?"() -> [if sensor yes? - is there a sensor here?]);
    //BIG FEATURE 3: camera feed
        //DON'T WORRY I WILL CHANGE THE TITLE I KNOW ITS DUMB ITS A PLACEHOLDER (so are the camera name and url in case you couldn't tell)
        teleop.addCamera("Robot Vision 👀", "camera name", "camera url");
        
    //Justin suggesion: Limelight info?
    }


}
