package frc.robot.sensors.Resolvers;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.AnalogInput;
import lombok.Getter;

@Getter
public class ResolverPointSlope {

    private AnalogInput resolver;
    private int channel;
    private double v1;
    private double v2;
    private double angle1;
    private double angle2;

    double r1 = 0;
    double r2 = 0;

    double t1 = 0;
    double t2 = 0;
    double count = 0;
    double v = 0;

    ArrayList<Double> average;

    public ResolverPointSlope(int channel, double v1, double v2, double angle1, double angle2) {
        this.channel = channel;
        this.v1 = v1;
        this.v2 = v2;
        this.angle1 = angle1;
        this.angle2 = angle2;
        resolver = new AnalogInput(channel);
        average = new ArrayList<>(20);
        for (int i = 0; i < 20; i++) {
            average.add(0.0);
        }

    }

    /**
     * @return Angle in radians
     */
    public double get() {
        return Math.toRadians(getD());
    }

    public double getV() {
        return resolver.getVoltage();
    }

    public double getD() {
        double v = resolver.getVoltage();
        double vSlope = (angle1 - angle2) / (v1 - v2);
        double angle = vSlope * (v-v1)+angle1;
        if (average.size() > 0) {
            for (int i = 0; i < average.size() - 1; i++) {
                average.set(i, average.get(i + 1));
            }
            average.set(average.size() - 1, angle);
        }
        return angle;
    }

    public double getDAverage() {
        if (average.size() > 0){
            return average.get(average.size() - 1);
        }
        return 0;
    }

    public double getVelocityRadians() {
        count++;
        getD();
        // System.out.println(average.size());
        r1 = Math.toRadians(getDAverage());
        if (count % 25 == 1) {

            t1 = System.currentTimeMillis();
            v = (r2 - r1) / (t2 - t1);
            t2 = t1;
            r2 = r1;
        }
        return v * 1000;
    }
}