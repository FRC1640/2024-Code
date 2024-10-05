package frc.robot.sensors.Gyro;

import java.util.OptionalDouble;
import java.util.Queue;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.drive.SparkMaxOdometryThread;

public class GyroIOSim implements GyroIO{

    Rotation2d angle = new Rotation2d();
    Supplier<Double> angularVelocityDegreesPerSecond;
    private final Queue<Double> rate;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    double offset;

    public GyroIOSim(Supplier<Double> angularVelocityDegreesPerSecond){
        yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> {
            return OptionalDouble.of(angle.getRadians());
        });
        rate = SparkMaxOdometryThread.getInstance().registerSignal(() -> {
            return OptionalDouble.of(angularVelocityDegreesPerSecond.get().doubleValue());
        });
        yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
        this.angularVelocityDegreesPerSecond = angularVelocityDegreesPerSecond;
    }
    @Override
    public void resetGyro(GyroIOInputs inputs) {
        offset = inputs.angleRadiansRaw;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        final double LOOP_PERIOD_SECS = 0.02;
        inputs.angleRadiansRaw = angle.getRadians() +  Math.toRadians(angularVelocityDegreesPerSecond.get()) * LOOP_PERIOD_SECS;
        angle = new Rotation2d(inputs.angleRadiansRaw);
        inputs.angularVelocityDegreesPerSecond = angularVelocityDegreesPerSecond.get();


        inputs.odometryRate = rate.stream().mapToDouble((Double value)->value).toArray();
        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRadians(value))
                .toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
        rate.clear();
    }
    @Override
    public double getActual(GyroIOInputs inputs){
        return inputs.angleRadiansRaw - offset;
    }
    @Override
    public double getOffset(){
        return offset;
    }

    @Override
    public void setOffset(double offset){
        this.offset = offset;
    }
}