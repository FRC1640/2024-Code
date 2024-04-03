package frc.robot.sensors.Gyro.Imported;

import edu.wpi.first.wpilibj.Timer;

public class Tracer 
{
    static String lastMsg = new String();
    static double lastMsgTimestamp = 0.0;

    static final double minDelta = 0.5;
    
    public static void Trace(String format, Object... args)
    {
        double msgTimestamp = Timer.getFPGATimestamp();
        double delta = msgTimestamp - lastMsgTimestamp;
        String msg = String.format(format, args);
        if ((lastMsg.compareTo(msg) != 0) ||
            (delta >= minDelta)) {
            System.out.printf(format, args);
            lastMsg = msg;
            lastMsgTimestamp = msgTimestamp;
        }
     }
};
