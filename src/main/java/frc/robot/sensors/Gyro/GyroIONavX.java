package frc.robot.sensors.Gyro;

import java.util.OptionalDouble;
import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.drive.SparkMaxOdometryThread;

public class GyroIONavX implements GyroIO {
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    double offset = 0;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final Queue<Double> rate;

    public GyroIONavX() {
        rate = SparkMaxOdometryThread.getInstance().registerSignal(() -> {
            boolean valid = gyro.isConnected() && !gyro.isCalibrating();
            if (valid) {
                return OptionalDouble.of(Math.toRadians(gyro.getRate()));
            } else {
                return OptionalDouble.empty();
            }
        });
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
        inputs.odometryRate = rate.stream().mapToDouble((Double value)->value).toArray();
        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRadians(value))
                .toArray(Rotation2d[]::new);

        inputs.accelX = gyro.getWorldLinearAccelX();
        inputs.accelY = gyro.getWorldLinearAccelY();
        inputs.accelZ = gyro.getWorldLinearAccelZ();
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
        rate.clear();
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