package frc.lib.sysid;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SimpleMotorSysidRoutine {

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
                        (Voltage volts) -> {
                            setVoltage.accept(volts.in(Volts));
                        }, log -> {
                            log.motor("Motor")
                                    .voltage(Volts.of(getVoltage.get()))
                                    .linearPosition(Meters.of(getPositionMeters.get()))
                                    .linearVelocity(MetersPerSecond.of(getVelocityMetersPerSecond.get()));
                        }, subsystem));
    }

    /**
     * Creates a new sysid routine with swerve modules
     *
     * @param setVoltage                 consumer for motor voltage
     * @param getVoltage                 supplier to get motor voltage
     * @param getPositionMeters          supplier for motor position
     * @param getVelocityMetersPerSecond supplier for motor velocity
     * @param subsystem                  subsystem for requirements
     * @param config                     config for sysid
     * @param motorName                  name of motor
     */
    public SysIdRoutine createNewRoutine(Consumer<Double> setVoltage, Supplier<Double> getVoltage,
            Supplier<Double> getPositionMeters, Supplier<Double> getVelocityMetersPerSecond, SubsystemBase subsystem,
            SysIdRoutine.Config config, String motorName) {
                return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            setVoltage.accept(volts.in(Volts));
                        }, log -> {
                            log.motor(motorName)
                                    .voltage(Volts.of(getVoltage.get()))
                                    .linearPosition(Meters.of(getPositionMeters.get()))
                                    .linearVelocity(MetersPerSecond.of(getVelocityMetersPerSecond.get()));
                        }, subsystem));
    }
}
