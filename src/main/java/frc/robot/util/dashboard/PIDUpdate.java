package frc.robot.util.dashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;

public class PIDUpdate{

    private static GenericEntry kP;
    private static GenericEntry kI;
    private static GenericEntry kD;
    private static GenericEntry kS;
    private static PIDController pid = new PIDController(0, 0, 0);

    public static void setEntries(GenericEntry p, GenericEntry i, GenericEntry d, GenericEntry s){
        kP = p;
        kI = i;
        kD = d;
        kS = s;
    }
    //method set var
    public static void setPID(PIDController controller){
        pid = controller;
    }

    public static PIDController getPID(){
        return pid;
    }

    public static void periodic() {
        pid.setP(kP.getDouble(0));
        pid.setI(kI.getDouble(0));
        pid.setD(kD.getDouble( 0));
        pid.setSetpoint(kS.getDouble(0));
    }
}
