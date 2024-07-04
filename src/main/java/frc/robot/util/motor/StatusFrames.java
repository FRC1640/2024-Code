package frc.robot.util.motor;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class StatusFrames {
    
    private int status0;
    private int status1;
    private int status2;
    private int status3;
    private int status4;
    private int status5;
    private int status6;

    public StatusFrames(int status0, int status1, int status2,
            int status3, int status4, int status5, int status6) {
        this.status0 = status0;
        this.status1 = status1;
        this.status2 = status2;
        this.status3 = status3;
        this.status4 = status4;
        this.status5 = status5;
        this.status6 = status6;
    }

    public void updateStatusFrames(CANSparkMax spark) {                
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, status0);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, status1);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, status2);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, status3);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, status4);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, status5);
        spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, status6);
    }
}
