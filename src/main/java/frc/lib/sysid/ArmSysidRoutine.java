package frc.lib.sysid;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ArmSysidRoutine {
	// Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
	private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
	// Mutable holder for unit-safe linear distance values, persisted to avoid
	// reallocation.
	private final MutableMeasure<Angle> distance = mutable(Radians.of(0));
	// Mutable holder for unit-safe linear velocity values, persisted to avoid
	// reallocation.
	private final MutableMeasure<Velocity<Angle>> velocity = mutable(RadiansPerSecond.of(0));

	/**
	 * Creates a new sysid routine with swerve modules
	 *
	 * @param setVoltage                 consumer for motor voltage
	 * @param getVoltage                 supplier to get motor voltage
	 * @param getPositionMeters          supplier for motor position
	 * @param getVelocityMetersPerSecond supplier for motor velocity
	 * @param subsystem                  subsystem for requirements
	 * @param config                     config for sysid
	 */
	public SysIdRoutine createNewRoutine(Consumer<Double> setVoltage, Supplier<Double> getVoltage,
			Supplier<Double> getPositionMeters, Supplier<Double> getVelocityMetersPerSecond, SubsystemBase subsystem,
			SysIdRoutine.Config config) {
		return new SysIdRoutine(
				config,
				new SysIdRoutine.Mechanism(
						(Measure<Voltage> volts) -> {
							setVoltage.accept(volts.in(Volts));
						}, log -> {
							log.motor("Motor")
									.voltage(appliedVoltage.mut_replace(
											getVoltage.get(),
											Volts))
									.angularPosition(distance.mut_replace(
											getPositionMeters.get(),
											Radians))
									.angularVelocity(velocity.mut_replace(
											getVelocityMetersPerSecond.get(),
											RadiansPerSecond));
						}, subsystem));
	}
}
