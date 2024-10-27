package frc.lib.drive.Module;

import java.util.OptionalDouble;
import java.util.Queue;

import org.opencv.calib3d.StereoBM;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.drive.SparkMaxOdometryThread;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PivotId;
import frc.robot.Constants.SwerveDriveDimensions;
import frc.robot.sensors.I2CResolvers.I2CMuxResolver;
import frc.robot.sensors.Resolvers.ResolverSlope;
import frc.robot.util.motor.SparkMaxConfigurer;

public class ModuleIOSparkMax implements ModuleIO {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steeringMotor;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;
    private final Queue<Double> driveVelocityQueue;

    private RelativeEncoder driveEncoder;
    private I2CMuxResolver steeringEncoder;

    // private final double kWheelRadius =
    // Constants.SwerveDriveDimensions.wheelRadius;
    private final double kDriveGearRatio = Constants.SwerveDriveDimensions.driveGearRatio;
    private ModuleInfo id;

    public ModuleIOSparkMax(ModuleInfo id) {

        this.id = id;
        driveMotor = SparkMaxConfigurer.configSpark(id.driveChannel,
                ModuleConstants.getSparkDefaultsDrive(id.reverseDrive));
        steeringMotor = SparkMaxConfigurer.configSpark(id.steerChannel,
                ModuleConstants.getSparkDefaultsSteer(id.reverseSteer));
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
        steeringEncoder = new I2CMuxResolver(id.mux, id.resolverAddress, id.resolverMuxChannel,
                id.angleOffset);

        driveVelocityQueue = SparkMaxOdometryThread.getInstance()
                .registerSignal(
                        () -> {
                            double value = driveEncoder.getVelocity();
                            if (driveMotor.getLastError() == REVLibError.kOk) {
                                return OptionalDouble.of(value);
                            } else {
                                return OptionalDouble.empty();
                            }
                        });
    }

    @Override
    public void setDriveIdleMode(boolean brake) {
        driveMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void resetSteer() {
        steeringMotor.getEncoder()
                .setPosition((360 - steeringEncoder.getD()) / 360 * SwerveDriveDimensions.steerGearRatio);
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
        inputs.steerAngleDegrees = 360
                - (steeringMotor.getEncoder().getPosition() / SwerveDriveDimensions.steerGearRatio * 360);
        inputs.steerAppliedVoltage = steeringMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        inputs.steerCurrentAmps = steeringMotor.getOutputCurrent();
        inputs.steerTempCelsius = steeringMotor.getMotorTemperature();
        // inputs.steerIdleModeIsBrake =
        // steeringMotor.getIdleMode().equals(IdleMode.kBrake);
        inputs.steerAngleRadians = Math.toRadians(inputs.steerAngleDegrees);
        // inputs.steerAngleVoltage = steeringEncoder.getV();
        inputs.steerAngleAbsolute = (steeringEncoder.getD());

        inputs.steerAngleRelative = 360
                - (steeringMotor.getEncoder().getPosition() / SwerveDriveDimensions.steerGearRatio * 360);

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

        inputs.odometryDrivePositionsMeters = drivePositionQueue.stream()
                .mapToDouble((Double value) -> -(value / kDriveGearRatio) * id.wheelRadius * 2 * Math.PI)
                .toArray();

        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(
                        360 - (value / SwerveDriveDimensions.steerGearRatio * 360)))
                .toArray(Rotation2d[]::new);

        inputs.driveVelocities = driveVelocityQueue.stream()
                .mapToDouble((Double value) -> -(value / kDriveGearRatio) / 60 * id.wheelRadius * 2 * Math.PI)
                .toArray();

        // inputs.accel = inputs.driveVelocityMetersPerSecond - lastV)

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
        driveVelocityQueue.clear();

        inputs.rawEncoderValue = steeringEncoder.getRawValue();
        // inputs.offset = steeringEncoder.getOffset();
        // inputs.LSBWeight = steeringEncoder.getLSBWeight();
    }

}
