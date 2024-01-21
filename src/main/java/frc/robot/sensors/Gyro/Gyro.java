package frc.robot.sensors.Gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.periodic.PeriodicBase;

public class Gyro extends PeriodicBase {
    private GyroIO io;
    private GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public Gyro(GyroIO io){
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Gyro", inputs);
    }

    public void reset() {
        io.resetGyro(inputs);
    }

    public Rotation2d getAngleRotation2d() {
        return new Rotation2d(io.getActual(inputs));
    }

    public double getRawAngleRadians(){
        return inputs.angleRadiansRaw;
    }

    public Rotation2d getRawAngleRotation2d(){
        return new Rotation2d(inputs.angleRadiansRaw);
    }

    public double getAngularVelDegreesPerSecond() {
        return inputs.angularVelocityDegreesPerSecond;
    }

    public boolean isTrustworthy() {
        return isConnected() && !isCalibrating();
    }

    public boolean isConnected() {
        return inputs.isConnected;
    }

    public boolean isCalibrating() {
        return inputs.isCalibrating;
    }
    public double getOffset(){
        return io.getOffset();
    }
}