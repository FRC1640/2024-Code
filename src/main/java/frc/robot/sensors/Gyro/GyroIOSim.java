package frc.robot.sensors.Gyro;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO{

    Rotation2d angle = new Rotation2d();
    Supplier<Double> angularVelocityDegreesPerSecond;

    public GyroIOSim(Supplier<Double> angularVelocityDegreesPerSecond){
        this.angularVelocityDegreesPerSecond = angularVelocityDegreesPerSecond;
    }
    @Override
    public void resetGyro() {
        angle = new Rotation2d(0);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        double LOOP_PERIOD_SECS = 0.02;
        inputs.angleDegrees = angle.getDegrees() + angularVelocityDegreesPerSecond.get() * LOOP_PERIOD_SECS;
        inputs.angleRadians = angle.getRadians() +  Math.toRadians(angularVelocityDegreesPerSecond.get()) * LOOP_PERIOD_SECS;
        angle = new Rotation2d(inputs.angleRadians);
        inputs.angularVelocityDegreesPerSecond = angularVelocityDegreesPerSecond.get();
    }
}
