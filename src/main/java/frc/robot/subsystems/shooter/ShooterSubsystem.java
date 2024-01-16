package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    ShooterIO io;
    public ShooterSubsystem(ShooterIO io){
        this.io = io;
    }
    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }
}
