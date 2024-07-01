package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SparkMaxDefaults;
import frc.robot.sensors.Resolvers.ResolverPointSlope;
import frc.robot.util.motor.SparkMaxConfiguration;
import frc.robot.util.motor.SparkMaxConfigurer;

public class ClimberIOSparkMax implements ClimberIO {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final DigitalInput leftProximitySensor;
    private final DigitalInput rightProximitySensor;
    private final ResolverPointSlope leftEncoder;
    private final ResolverPointSlope rightEncoder;

    public ClimberIOSparkMax() {
        leftMotor = SparkMaxConfigurer.configSpark(ClimberConstants.leftCanID,
                new SparkMaxConfiguration(
                    IdleMode.kBrake, 
                    false,
                    SparkMaxDefaults.limitSwitch,
                    SparkMaxDefaults.limSwitchType,
                    80,
                    SparkMaxDefaults.encoderMeasurementPeriod,
                    SparkMaxDefaults.encoderAverageDepth,
                    SparkMaxDefaults.canTimeout));
        rightMotor = SparkMaxConfigurer.configSpark(ClimberConstants.rightCanID,
                new SparkMaxConfiguration(
                    IdleMode.kBrake, 
                    false,
                    SparkMaxDefaults.limitSwitch,
                    SparkMaxDefaults.limSwitchType,
                    80,
                    SparkMaxDefaults.encoderMeasurementPeriod,
                    SparkMaxDefaults.encoderAverageDepth,
                    SparkMaxDefaults.canTimeout));
        leftProximitySensor = new DigitalInput(ClimberConstants.leftProximityChannel);
        rightProximitySensor = new DigitalInput(ClimberConstants.rightProximityChannel);

        Constants.updateStatusFrames(leftMotor, 100, 20, 20, 500, 500, 500, 500);
        Constants.updateStatusFrames(rightMotor, 100, 20, 20, 500, 500, 500, 500);
        leftEncoder = new ResolverPointSlope(ClimberConstants.leftClimberResolver, 1.327, 2.356, 1, 76);
        rightEncoder = new ResolverPointSlope(ClimberConstants.rightClimberResolver, 3.637, 2.585, 11, 84); //note: to create resolver constants class for min/max
    }

    @Override
    public void setLeftSpeedVoltage(double lVoltage){
        double speedClamped = lVoltage;
        speedClamped = clampSpeeds(leftEncoder.getD(), speedClamped);
        leftMotor.setVoltage(speedClamped);
    }

    @Override
    public void setRightSpeedVoltage(double rVoltage){
        double speedClamped = rVoltage;
        speedClamped = clampSpeeds(rightEncoder.getD(), speedClamped);
        rightMotor.setVoltage(speedClamped);
    }

    @Override
    public void setLeftSpeedPercent(double speed){
        speed = clampSpeeds(leftEncoder.getD(), speed);
        leftMotor.set(speed);
    }

    @Override
    public void setRightSpeedPercent(double speed){
        speed = clampSpeeds(rightEncoder.getD(), speed);
        rightMotor.set(speed);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs){
        inputs.leftSpeedPercent = leftMotor.getAppliedOutput();
        inputs.leftAppliedVoltage = leftMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();
        inputs.leftTempCelcius = leftMotor.getMotorTemperature();
        inputs.leftClimberPositionDegrees = leftEncoder.getD();
        inputs.leftClimberVoltage = leftEncoder.getV();
        inputs.leftProximitySensor = !(leftProximitySensor.get());

        inputs.rightSpeedPercent = rightMotor.getAppliedOutput();
        inputs.rightAppliedVoltage = rightMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
        inputs.rightTempCelcius = rightMotor.getMotorTemperature();
        inputs.rightClimberPositionDegrees = rightEncoder.getD();
        inputs.rightClimberVoltage = rightEncoder.getV();
        inputs.rightProximitySensor = !(rightProximitySensor.get());
    }
}
