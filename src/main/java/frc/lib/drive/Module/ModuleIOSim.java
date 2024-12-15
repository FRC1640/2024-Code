package frc.lib.drive.Module;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Queue;

import com.revrobotics.REVLibError;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.drive.SparkMaxOdometryThread;
import frc.robot.Constants.SimulationConstants;
import frc.robot.Constants.SwerveDriveDimensions;

public class ModuleIOSim implements ModuleIO {

        private static final double LOOP_PERIOD_SECS = 0.02;
        LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1),
                        0.00019125, SwerveDriveDimensions.driveGearRatio);
        LinearSystem<N2, N1, N2> plant1 = LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.00019125,
                        SwerveDriveDimensions.steerGearRatio);
        private DCMotorSim driveSim = new DCMotorSim(plant, DCMotor.getNeoVortex(1),
                        SwerveDriveDimensions.driveGearRatio, 0.00019125); // Drive motor sim using moi

        private DCMotorSim steerSim = new DCMotorSim(plant1, DCMotor.getNeo550(1),
                        SwerveDriveDimensions.steerGearRatio, 0.002174375); // Steer motor sim using moi

        private double driveAppliedVolts = 0.0;
        private double steerAppliedVolts = 0.0;

        private final Queue<Double> timestampQueue;
        private final Queue<Double> drivePositionQueue;
        private final Queue<Double> turnPositionQueue;
        private final Queue<Double> driveVelocityQueue;

        double drivePos = 0;
        double steerPos = 0;

        public ModuleIOSim() {
                timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
                drivePositionQueue = SparkMaxOdometryThread.getInstance()
                                .registerSignal(
                                                () -> OptionalDouble.of(drivePos));

                turnPositionQueue = SparkMaxOdometryThread.getInstance()
                                .registerSignal(
                                                () -> OptionalDouble.of(steerPos));
                driveVelocityQueue = SparkMaxOdometryThread.getInstance()
                                .registerSignal(
                                                () -> {
                                                        double value = driveSim.getAngularVelocityRadPerSec();
                                                        return OptionalDouble.of(value);
                                                });
        }

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
                steerAppliedVolts = -12 * percentage;
                steerSim.setInputVoltage(-12 * percentage);
        }

        @Override
        public void setSteerVoltage(double voltage) {
                steerAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
                steerSim.setInputVoltage(-steerAppliedVolts);
        }

        @Override
        public void updateInputs(ModuleIOInputs inputs) {

                driveSim.update(LOOP_PERIOD_SECS);
                steerSim.update(LOOP_PERIOD_SECS);

                inputs.drivePositionMeters -= ((driveSim.getAngularVelocityRPM()) / 60) * 2 * Math.PI
                                * Units.inchesToMeters(3.7432661290322 / 2) * LOOP_PERIOD_SECS;

                drivePos = inputs.drivePositionMeters;

                steerPos = inputs.steerAngleDegrees;

                inputs.driveVelocityMetersPerSecond = -((driveSim.getAngularVelocityRPM()) / 60) * 2 * Math.PI
                                * Units.inchesToMeters(3.7432661290322 / 2);
                inputs.driveAppliedVoltage = -driveAppliedVolts;
                inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();
                inputs.driveTempCelsius = SimulationConstants.roomTempCelsius;

                inputs.steerAngleDegrees += (steerSim.getAngularVelocityRPM() * 360 / 60) * LOOP_PERIOD_SECS;
                inputs.steerRPS = steerSim.getAngularVelocityRPM() / 60;
                inputs.steerAppliedVoltage = steerAppliedVolts;
                inputs.steerCurrentAmps = steerSim.getCurrentDrawAmps();
                inputs.steerTempCelsius = SimulationConstants.roomTempCelsius;
                inputs.steerAngleRadians = Math.toRadians(inputs.steerAngleDegrees);

                inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

                inputs.odometryDrivePositionsMeters = drivePositionQueue.stream()
                                .mapToDouble((Double value) -> value)
                                .toArray();

                inputs.odometryTurnPositions = turnPositionQueue.stream()
                                .map((Double value) -> Rotation2d.fromDegrees(
                                                value))
                                .toArray(Rotation2d[]::new);

                inputs.driveVelocities = driveVelocityQueue.stream().mapToDouble(
                                (Double value) -> value * 2 * Math.PI * Units.inchesToMeters(3.7432661290322 / 2))
                                .toArray();

                timestampQueue.clear();
                drivePositionQueue.clear();
                turnPositionQueue.clear();
                driveVelocityQueue.clear();

        }

}
