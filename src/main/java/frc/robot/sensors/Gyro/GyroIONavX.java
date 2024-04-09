package frc.robot.sensors.Gyro;

import java.util.OptionalDouble;
import java.util.Queue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.lib.drive.SparkMaxOdometryThread;
import frc.robot.sensors.Gyro.Imported.AHRS;

public class GyroIONavX implements GyroIO {
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    double offset = 0;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIONavX() {
        yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkMaxOdometryThread.getInstance()
                .registerSignal(
                        () -> {
                            boolean valid = gyro.isConnected() && !gyro.isCalibrating();
                            if (valid) {
                                return OptionalDouble.of(gyro.getRotation2d().getRadians());
                            } else {
                                return OptionalDouble.empty();
                            }
                        });
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {

        inputs.isConnected = gyro.isConnected();
        inputs.isCalibrating = gyro.isCalibrating();
        inputs.angleRadiansRaw = gyro.getRotation2d().getRadians();
        inputs.angularVelocityDegreesPerSecond = gyro.getRate();
        inputs.angleDegreesRaw = Math.toDegrees(inputs.angleRadiansRaw);

        inputs.displacementX = gyro.getDisplacementX();
        inputs.displacementY = gyro.getDisplacementY();

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRadians(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void resetGyro(GyroIOInputs inputs) {
        offset = inputs.angleRadiansRaw;
    }

    @Override
    public double getActual(GyroIOInputs inputs) {
        return inputs.angleRadiansRaw - offset;
    }

    @Override
    public double getOffset() {
        return offset;
    }

    @Override
    public void setOffset(double offset) {
        this.offset = offset;
    }
}