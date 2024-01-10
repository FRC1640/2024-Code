package frc.util.periodics;

public abstract class PeriodicBase {

    public PeriodicBase() {
        PeriodicScheduler.getInstance().addPeriodic(this);
    }

    public abstract void periodic();
    
}
