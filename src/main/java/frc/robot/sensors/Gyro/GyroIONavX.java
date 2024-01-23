package frc.robot.sensors.Gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO{
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    double offset = 0;
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        
        inputs.isConnected = gyro.isConnected();
        inputs.isCalibrating = gyro.isCalibrating();
        inputs.angleRadiansRaw = gyro.getRotation2d().getRadians();
        inputs.angularVelocityDegreesPerSecond = gyro.getRate();
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
}