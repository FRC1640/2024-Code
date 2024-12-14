package frc.lib.sysid;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drive.Module.Module;
import static edu.wpi.first.units.Units.*;

public class SwerveDriveSysidRoutine {

    /**
     * Creates a new sysid routine with swerve modules
     *
     * @param fl        front left module
     * @param fr        front right module
     * @param bl        back left module
     * @param br        front right module
     * @param subsystem subsystem for requirments
     * @param config    config for sysid
     */
    public SysIdRoutine createNewRoutine(Module fl, Module fr, Module bl, Module br, SubsystemBase subsystem,
            SysIdRoutine.Config config) {
        return new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            fl.setDriveVoltage(volts.in(Volts));
                            fr.setDriveVoltage(-volts.in(Volts));
                            bl.setDriveVoltage(volts.in(Volts));
                            br.setDriveVoltage(-volts.in(Volts));
                        }, log -> {
                            log.motor("frontLeft")
                                .voltage(Volts.of(fl.getDriveVoltage()))
                                .linearPosition(Meters.of(fl.getPosition().distanceMeters))
                                .linearVelocity(MetersPerSecond.of(fl.getVelocity()));
                            log.motor("frontRight")
                                .voltage(Volts.of(-fr.getDriveVoltage()))
                                .linearPosition(Meters.of(fr.getPosition().distanceMeters))
                                .linearVelocity(MetersPerSecond.of(fr.getVelocity()));
                            log.motor("backLeft")
                                .voltage(Volts.of(bl.getDriveVoltage()))
                                .linearPosition(Meters.of(bl.getPosition().distanceMeters))
                                .linearVelocity(MetersPerSecond.of(bl.getVelocity()));
                            log.motor("backRight")
                                .voltage(Volts.of(-br.getDriveVoltage()))
                                .linearPosition(Meters.of(br.getPosition().distanceMeters))
                                .linearVelocity(MetersPerSecond.of(br.getVelocity()));
                        }, subsystem));
    }
}
