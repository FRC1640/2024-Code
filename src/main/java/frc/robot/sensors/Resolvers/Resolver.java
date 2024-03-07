package frc.robot.sensors.Resolvers;

import edu.wpi.first.wpilibj.AnalogInput;
import lombok.Getter;

@Getter
public class Resolver {

    public double minV;
    public double maxV;
    private double offset;
    private boolean reverse;

    private AnalogInput resolver;

    public Resolver (int channel, double minV, double maxV, double offset, boolean reverse) {
        resolver = new AnalogInput(channel);
        this.minV = minV;
        this.maxV = maxV;
        this.offset = offset;
        this.reverse = reverse;
    }

    public String toString() {
        return String.format("%f, %f", minV, maxV);
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
        updateBounds(v);

        double vSlope = 360.0 / ((maxV - minV));
		double vOffset = -vSlope * minV;
        double angle = (((vSlope * v + vOffset) - offset) + 360) % 360;

        return reverse ? 360 - angle : angle;
    }

    private void updateBounds (double v) {
        // System.out.println("min: " + minV + " max: " + maxV);
        minV = Math.min(minV, v);
        maxV = Math.max(maxV, v);
    }
}