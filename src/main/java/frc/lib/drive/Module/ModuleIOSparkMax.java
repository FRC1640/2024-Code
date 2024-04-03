package frc.lib.drive.Module;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ModuleConstants;
import frc.robot.sensors.Resolvers.Resolver;
import frc.robot.sensors.Resolvers.ResolverPointSlope;
import frc.robot.sensors.Resolvers.ResolverSlope;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.RobotController;

public class ModuleIOSparkMax implements ModuleIO{
    private final CANSparkMax driveMotor;
	private final CANSparkMax steeringMotor;

    
	private RelativeEncoder driveEncoder;
	public ResolverSlope steeringEncoder;
    // private final double kWheelRadius = Constants.SwerveDriveDimensions.wheelRadius;
    private final double kDriveGearRatio = Constants.SwerveDriveDimensions.driveGearRatio;
    private ModuleInfo id;



    public ModuleIOSparkMax(ModuleInfo id){
        this.id = id;
        driveMotor = new CANSparkMax(id.driveChannel, MotorType.kBrushless);
        steeringMotor = new CANSparkMax(id.steerChannel, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(60);
        driveMotor.getEncoder().setMeasurementPeriod(8);
        driveMotor.getEncoder().setAverageDepth(2);
		steeringMotor.setIdleMode(IdleMode.kCoast);
		steeringMotor.setSmartCurrentLimit(40);
		driveMotor.setIdleMode(IdleMode.kCoast);

        Constants.updateStatusFrames(driveMotor, 100, 20, 20, 500, 500, 500, 500);
        Constants.updateStatusFrames(steeringMotor, 100, 20, 20, 500, 500, 500, 500);
        driveEncoder = driveMotor.getEncoder();
		steeringEncoder = new ResolverSlope(id.resolverChannel, 3.177, 4.43,
				180.0,90.0,id.angleOffset);

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
    public void setBrakeMode(boolean brake){
        if (brake){
            driveMotor.setIdleMode(IdleMode.kBrake);
            steeringMotor.setIdleMode(IdleMode.kBrake);
        }
        else{
            driveMotor.setIdleMode(IdleMode.kCoast);
            steeringMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionMeters = -(driveEncoder.getPosition() / kDriveGearRatio) * id.wheelRadius * 2 * Math.PI;
        inputs.driveVelocityMetersPerSecond =  -((driveEncoder.getVelocity() / kDriveGearRatio) / 60) * 2 * Math.PI * id.wheelRadius;
        inputs.driveAppliedVoltage = driveMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelsius = driveMotor.getMotorTemperature();
        // inputs.driveIdleModeIsBrake = driveMotor.getIdleMode().equals(IdleMode.kBrake);

        inputs.steerAngleDegrees = steeringEncoder.getD();
        inputs.steerAppliedVoltage = steeringMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.steerCurrentAmps = steeringMotor.getOutputCurrent();
        inputs.steerTempCelsius = steeringMotor.getMotorTemperature();
        // inputs.steerIdleModeIsBrake = steeringMotor.getIdleMode().equals(IdleMode.kBrake);
        inputs.steerAngleRadians = Math.toRadians(inputs.steerAngleDegrees);
        inputs.steerAngleVoltage = steeringEncoder.getV();
    }

    
}
