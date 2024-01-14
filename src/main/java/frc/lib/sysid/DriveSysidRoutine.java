package frc.lib.sysid;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Module;

public class DriveSysidRoutine {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Distance> distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Distance>> velocity = mutable(MetersPerSecond.of(0));

    public SysIdRoutine createNewRoutineSwerve(Module fl, Module fr, Module bl, Module br, SubsystemBase subsystem, SysIdRoutine.Config config) {
        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            fl.setDriveVoltage(volts.in(Volts));
                            fr.setDriveVoltage(volts.in(Volts));
                            bl.setDriveVoltage(volts.in(Volts));
                            br.setDriveVoltage(volts.in(Volts));
                        }, log -> {
                            log.motor("frontLeft")
                                    .voltage(appliedVoltage.mut_replace(fl.getDriveVoltage(),
                                            Volts))
                                    .linearPosition(distance.mut_replace(fl.getPosition().distanceMeters, Meters))
                                    .linearVelocity(velocity.mut_replace(fl.getVelocity(), MetersPerSecond));
                            log.motor("frontRight")
                                    .voltage(appliedVoltage.mut_replace(fr.getDriveVoltage(),
                                            Volts))
                                    .linearPosition(distance.mut_replace(fr.getPosition().distanceMeters, Meters))
                                    .linearVelocity(velocity.mut_replace(fr.getVelocity(), MetersPerSecond));
                            log.motor("backLeft")
                                    .voltage(appliedVoltage.mut_replace(bl.getDriveVoltage(),
                                            Volts))
                                    .linearPosition(distance.mut_replace(bl.getPosition().distanceMeters, Meters))
                                    .linearVelocity(velocity.mut_replace(bl.getVelocity(), MetersPerSecond));
                            log.motor("backRight")
                                    .voltage(appliedVoltage.mut_replace(br.getDriveVoltage(),
                                            Volts))
                                    .linearPosition(distance.mut_replace(br.getPosition().distanceMeters, Meters))
                                    .linearVelocity(velocity.mut_replace(br.getVelocity(), MetersPerSecond));
                        }, subsystem));
    }
}
