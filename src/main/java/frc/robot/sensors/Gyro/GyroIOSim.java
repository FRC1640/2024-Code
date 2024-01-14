package frc.robot.sensors.Gyro;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO{

    Rotation2d angle = new Rotation2d();
    Supplier<Double> angularVelocityDegreesPerSecond;

    double offset;

    public GyroIOSim(Supplier<Double> angularVelocityDegreesPerSecond){
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
    }
    @Override
    public double getActual(GyroIOInputs inputs){
        return inputs.angleRadiansRaw - offset;
    }
}