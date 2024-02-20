package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import lombok.Getter;

@Getter
public class ResolverSlope {

    private AnalogInput resolver;
    private int channel;
    private double v1;
    private double v2;
    private double angle1;
    private double angle2;
    private double offset;

    public ResolverSlope (int channel, double v1, double v2, double angle1, double angle2, double offset) {
        this.channel = channel;
        this.v1 = v1;
        this.v2 = v2;
        this.angle1 = angle1;
        this.angle2 = angle2;
        this.offset = offset;
        resolver = new AnalogInput(channel);

    }

    /**
     * @return Angle in radians
     */
    public double get () {
        return Math.toRadians(getD());
    }
    public double getV(){
        return resolver.getVoltage();
    }

    public double getD () {
        double v = resolver.getVoltage();

        double vSlope = (angle1 - angle2) / (v1-v2); 
        double angle = vSlope * v + offset;

        return angle;
    }
}