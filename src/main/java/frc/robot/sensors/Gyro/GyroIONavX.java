package frc.robot.sensors.Gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO{
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    @Override
    public void resetGyro() {
        gyro.zeroYaw();
        gyro.reset();
        
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = gyro.isConnected();
        inputs.isCalibrating = gyro.isCalibrating();
        inputs.angleDegrees = gyro.getRotation2d().getDegrees();
        inputs.angleRadians = gyro.getRotation2d().getRadians();
        inputs.angularVelocityDegreesPerSecond = gyro.getRate();
    }
    
}
