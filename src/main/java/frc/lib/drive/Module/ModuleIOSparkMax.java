package frc.lib.drive.Module;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.sensors.Resolver;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.RobotController;

public class ModuleIOSparkMax implements ModuleIO{
    private final CANSparkMax driveMotor;
	private final CANSparkMax steeringMotor;

    
	private RelativeEncoder driveEncoder;
	public Resolver steeringEncoder;

    private final double kWheelRadius = Constants.SwerveDriveDimensions.wheelRadius;
    private final double kDriveGearRatio = Constants.SwerveDriveDimensions.driveGearRatio;



    public ModuleIOSparkMax(ModuleInfo id){
        driveMotor = new CANSparkMax(id.driveChannel, MotorType.kBrushless);
        steeringMotor = new CANSparkMax(id.steerChannel, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(60);
		steeringMotor.setIdleMode(IdleMode.kCoast);
		steeringMotor.setSmartCurrentLimit(40);
		driveMotor.setIdleMode(IdleMode.kBrake);
        steeringMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
		steeringMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
		steeringMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
		driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
        driveEncoder = driveMotor.getEncoder();
		steeringEncoder = new Resolver(id.resolverChannel, ModuleConstants.minVoltage, ModuleConstants.maxVoltage,
				id.angleOffset, id.reverseAngle);

        driveMotor.setInverted(id.reverseDrive);
        steeringMotor.setInverted(id.reverseSteer);
    }


    @Override
    public void setDriveIdleMode(boolean brake) {
        driveMotor.setIdleMode(brake ? IdleMode.kBrake:IdleMode.kCoast);
    }


    @Override
    public void setDrivePercentage(double percentage) {
        driveMotor.set(percentage);
    }


    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }


    @Override
    public void setSteerIdleMode(boolean brake) {
        steeringMotor.setIdleMode(brake ? IdleMode.kBrake:IdleMode.kCoast);
    }


    @Override
    public void setSteerPercentage(double percentage) {
        steeringMotor.set(percentage);
    }


    @Override
    public void setSteerVoltage(double voltage) {
        steeringMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionMeters = -(driveEncoder.getPosition() / kDriveGearRatio) * kWheelRadius * 2 * Math.PI;
        inputs.driveVelocityMetersPerSecond =  -((driveEncoder.getVelocity() / kDriveGearRatio) / 60) * 2 * Math.PI * kWheelRadius;
        inputs.driveAppliedVoltage = driveMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelsius = driveMotor.getMotorTemperature();
        inputs.driveIdleModeIsBrake = driveMotor.getIdleMode().equals(IdleMode.kBrake);

        inputs.steerAngleDegrees = steeringEncoder.getD();
        inputs.steerAppliedVoltage = steeringMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.steerCurrentAmps = steeringMotor.getOutputCurrent();
        inputs.steerTempCelsius = steeringMotor.getMotorTemperature();
        inputs.steerIdleModeIsBrake = steeringMotor.getIdleMode().equals(IdleMode.kBrake);
        inputs.steerAngleRadians = Math.toRadians(inputs.steerAngleDegrees);
    }

    
}
