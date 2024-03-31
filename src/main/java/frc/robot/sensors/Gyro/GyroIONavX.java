package frc.robot.sensors.Gyro;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.sensors.Gyro.Imported.AHRS;

public class GyroIONavX implements GyroIO{
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    double offset = 0;
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        
        inputs.isConnected = gyro.isConnected();
        inputs.isCalibrating = gyro.isCalibrating();
        inputs.angleRadiansRaw = gyro.getRotation2d().getRadians();
        inputs.angularVelocityDegreesPerSecond = gyro.getRate();
        inputs.angleDegreesRaw = Math.toDegrees(inputs.angleRadiansRaw);

        inputs.displacementX = gyro.getDisplacementX();
        inputs.displacementY = gyro.getDisplacementY();
    }
    @Override
    public void resetGyro(GyroIOInputs inputs) {
        offset = inputs.angleRadiansRaw;
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