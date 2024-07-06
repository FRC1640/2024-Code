package frc.lib.drive.Module;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import frc.lib.drive.SparkMaxOdometryThread;
import frc.robot.Constants;
import frc.robot.Constants.SparkMaxDefaults;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.sensors.Resolvers.ResolverSlope;
import frc.robot.util.motor.SparkMaxConfiguration;
import frc.robot.util.motor.SparkMaxConfigurer;
import frc.robot.util.motor.StatusFrames;

import java.util.OptionalDouble;
import java.util.Queue;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;

public class ModuleIOSparkMax implements ModuleIO {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steeringMotor;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private RelativeEncoder driveEncoder;
    public ResolverSlope steeringEncoder;
    // private final double kWheelRadius =
    // Constants.SwerveDriveDimensions.wheelRadius;
    private final double kDriveGearRatio = Constants.SwerveDriveDimensions.driveGearRatio;
    private ModuleInfo id;
    private String name;

    public ModuleIOSparkMax(ModuleInfo id) {

        

        this.id = id;
        driveMotor = SparkMaxConfigurer.configSpark(
            id.driveChannel,
            new SparkMaxConfiguration(
                SparkMaxDefaults.idleMode,
                id.reverseDrive,
                SparkMaxDefaults.limitSwitch,
                SparkMaxDefaults.limSwitchType,
                80,
                8,
                2,
                250,
                new StatusFrames(100, 20, (int) (1000 / SwerveDriveDimensions.odometryFrequency),
                    500, 500, 500, 500)));
        steeringMotor = SparkMaxConfigurer.configSpark(
            id.steerChannel,
            new SparkMaxConfiguration(
                SparkMaxDefaults.idleMode,
                id.reverseSteer,
                SparkMaxDefaults.limitSwitch,
                SparkMaxDefaults.limSwitchType,
                40,
                8,
                2,
                250,
                new StatusFrames(100, 20, (int) (1000 / SwerveDriveDimensions.odometryFrequency),
                    500, 500, 500, 500)));
        timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkMaxOdometryThread.getInstance()
                .registerSignal(
                        () -> {
                            double value = driveEncoder.getPosition();
                            if (driveMotor.getLastError() == REVLibError.kOk) {
                                return OptionalDouble.of(value);
                            } else {
                                return OptionalDouble.empty();
                            }
                        });

        turnPositionQueue = SparkMaxOdometryThread.getInstance()
                        .registerSignal(
                        () -> {
                            double value = steeringMotor.getEncoder().getPosition();
                            if (steeringMotor.getLastError() == REVLibError.kOk) {
                                return OptionalDouble.of(value);
                            } else {
                                return OptionalDouble.empty();
                            }
                        });
        driveEncoder = driveMotor.getEncoder();
        steeringEncoder = new ResolverSlope(id.resolverChannel, 3.125, 4.375,
                180.0, 90.0, id.angleOffset);
    }

    @Override
    public void setDriveIdleMode(boolean brake) {
        driveMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override 
    public void resetSteer(){
        steeringMotor.getEncoder().setPosition((360-steeringEncoder.getD()) / 360 * SwerveDriveDimensions.steerGearRatio);
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
        steeringMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
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
    public void setBrakeMode(boolean brake) {
        if (brake) {
            driveMotor.setIdleMode(IdleMode.kBrake);
            steeringMotor.setIdleMode(IdleMode.kBrake);
        } else {
            driveMotor.setIdleMode(IdleMode.kCoast);
            steeringMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double lastV = inputs.driveVelocityMetersPerSecond;
        inputs.drivePositionMeters = -(driveEncoder.getPosition() / kDriveGearRatio) * id.wheelRadius * 2 * Math.PI;
        inputs.driveVelocityMetersPerSecond = -((driveEncoder.getVelocity() / kDriveGearRatio) / 60) * 2 * Math.PI
                * id.wheelRadius;
        inputs.driveAppliedVoltage = driveMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelsius = driveMotor.getMotorTemperature();
        // inputs.driveIdleModeIsBrake =
        // driveMotor.getIdleMode().equals(IdleMode.kBrake);

        // inputs.steerAngleDegrees = steeringEncoder.getD();
        inputs.steerAngleDegrees = 360 - (steeringMotor.getEncoder().getPosition() / SwerveDriveDimensions.steerGearRatio * 360);
        inputs.steerAppliedVoltage = steeringMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.steerCurrentAmps = steeringMotor.getOutputCurrent();
        inputs.steerTempCelsius = steeringMotor.getMotorTemperature();
        // inputs.steerIdleModeIsBrake =
        // steeringMotor.getIdleMode().equals(IdleMode.kBrake);
        inputs.steerAngleRadians = Math.toRadians(inputs.steerAngleDegrees);
        inputs.steerAngleVoltage = steeringEncoder.getV();
        inputs.steerAngleAbsolute = (steeringEncoder.getD());

        inputs.steerAngleRelative = 360 - (steeringMotor.getEncoder().getPosition() / SwerveDriveDimensions.steerGearRatio * 360);

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

        inputs.odometryDrivePositionsMeters = drivePositionQueue.stream()
                .mapToDouble((Double value) ->  -(value / kDriveGearRatio) * id.wheelRadius * 2 * Math.PI)
                .toArray();

        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(
                    360 - (value / SwerveDriveDimensions.steerGearRatio * 360)))
                .toArray(Rotation2d[]::new);

        // inputs.accel = inputs.driveVelocityMetersPerSecond - lastV)

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

}
