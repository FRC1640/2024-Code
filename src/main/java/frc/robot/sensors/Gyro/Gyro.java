package frc.robot.sensors.Gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.util.periodics.PeriodicBase;

public class Gyro extends PeriodicBase{
    GyroIO io;
    GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
    public Gyro(GyroIO io){
        this.io = io;
    }
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Gyro", inputs);
    }
    public void reset() {
        io.resetGyro();
    }

    public double getAngleDegrees() {
        return inputs.angleDegrees;
    }

    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees((getAngleDegrees()));
    }

    public double getAngularVelDegreesPerSecond() {
        return inputs.angularVelocityDegreesPerSecond;
    }
}
