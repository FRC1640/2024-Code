package frc.robot.util.dashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import frc.lib.periodic.PeriodicBase;

public class PIDUpdate{

    private static GenericEntry kP;
    private static GenericEntry kI;
    private static GenericEntry kD;
    private static GenericEntry kSetpoint;
    private static PIDController pid;

    public static void setEntries(GenericEntry p, GenericEntry i, GenericEntry d, GenericEntry setpoint){
        kP = p;
        kI = i;
        kD = d;
        kSetpoint = setpoint;
    }
    //method set var
    public static void setPID(PIDController controller){
        pid = controller;
    }

    public static void periodic() {
        pid.setP(kP.getDouble(0));
        pid.setI(kI.getDouble(0));
        pid.setD(kD.getDouble(0));
        pid.setSetpoint(kSetpoint.getDouble(0));
    }
}
