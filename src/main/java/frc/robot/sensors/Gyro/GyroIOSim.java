package frc.robot.sensors.Gyro;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class GyroIOSim implements GyroIO{

    Rotation2d angle = new Rotation2d();
    CommandXboxController controller;
    double roll = 0;
    double pitch = 0;
    Supplier<Double> angularVelocityDegreesPerSecond;

    double offset;

    public GyroIOSim(Supplier<Double> angularVelocityDegreesPerSecond, CommandXboxController controller){
        this.controller = controller;
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
        inputs.rotation3d = new Rotation3d(roll, pitch, angle.getRadians());
        inputs.roll = roll;
        if (controller.povUp().getAsBoolean()){
            pitch += 0.01;
        }
        if (controller.povDown().getAsBoolean()){
            pitch -= 0.01;
        }
        if (controller.povRight().getAsBoolean()){
            roll += 0.01;
        }
        if (controller.povLeft().getAsBoolean()){
            roll -= 0.01;
        }
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
    @Override
    public Rotation3d getRotation3d() {
        return new Rotation3d(roll, pitch, angle.getRadians());
    }
}