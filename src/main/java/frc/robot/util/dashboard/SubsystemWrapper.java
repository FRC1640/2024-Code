package frc.robot.util.dashboard;

public class SubsystemWrapper<S> {
    
    private S subsystem;

    public SubsystemWrapper(S subsystem) {
        this.subsystem = subsystem;
    }

    public S unwrapSubsystem() {
        return subsystem;
    }
}
