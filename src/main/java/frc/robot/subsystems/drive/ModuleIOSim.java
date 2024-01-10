package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Constants.SwerveDriveDimensions;

public class ModuleIOSim implements ModuleIO{

    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), SwerveDriveDimensions.driveGearRatio, 0.025);
    private DCMotorSim steerSim = new DCMotorSim(DCMotor.getNEO(1), SwerveDriveDimensions.steerGearRatio, 0.004);
    private double driveAppliedVolts = 0.0;
    private double steerAppliedVolts = 0.0;

    @Override
    public void setDrivePercentage(double percentage) {
        driveAppliedVolts = 12 * percentage;
        driveSim.setInputVoltage(12 * percentage);
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setSteerPercentage(double percentage) {
        steerAppliedVolts = 12 * percentage;
        steerSim.setInputVoltage(12 * percentage);
    }

    @Override
    public void setSteerVoltage(double voltage) {
        steerAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
        steerSim.setInputVoltage(steerAppliedVolts);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        driveSim.update(LOOP_PERIOD_SECS);
        steerSim.update(LOOP_PERIOD_SECS);

        System.out.println(driveSim.getAngularPositionRotations());

        inputs.drivePositionMeters += ((driveSim.getAngularVelocityRPM()) / 60) * 2 * Math.PI * SwerveDriveDimensions.wheelRadius * LOOP_PERIOD_SECS;
        inputs.driveVelocityMetersPerSecond =  ((driveSim.getAngularVelocityRPM()) / 60) * 2 * Math.PI * SwerveDriveDimensions.wheelRadius;
        inputs.driveAppliedVoltage = driveAppliedVolts;
        inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();
        inputs.driveTempCelsius = SimulationConstants.roomTempCelsius;

        inputs.steerAngleDegrees = Math.toDegrees(steerSim.getAngularPositionRad());
        inputs.steerAppliedVoltage = steerAppliedVolts;
        inputs.steerCurrentAmps = steerSim.getCurrentDrawAmps();
        inputs.steerTempCelsius = SimulationConstants.roomTempCelsius;

    }
    
}
