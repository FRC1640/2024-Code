// package frc.robot.util.dashboard;

// import java.util.function.DoubleConsumer;

// import edu.wpi.first.networktables.GenericEntry;
// import frc.robot.subsystems.climber.ClimberSubsystem;
// import frc.robot.subsystems.targeting.TargetingSubsystem;

// public class MotorUpdate {
    
//     private GenericEntry motorSpeed;
//     private DoubleConsumer setter;
//     private LimitedSubsystem limitedSubsystem;
//     private TargetingFunction targetingFunction;
//     private GenericEntry toggleEntry;
//     private TargetingSubsystem targetingSubsystem;
//     private ClimberSubsystem climberSubsystem;

//     private enum LimitedSubsystem {
//         CLIMBER, TARGETING, UNLIMITED
//     }

//     public enum TargetingFunction {
//         ANGLER, EXTENSION
//     }

//     /**
//      * Periodically updates the percent output of a motor or group of motors.
//      * 
//      * @param motorSpeed NetworkTables {@code GenericEntry} from which to get the speed.
//      * @param setter {@code DoubleConsumer} from a subsystem to use to set the motor speeds.
//      */
//     public MotorUpdate(GenericEntry motorSpeed, DoubleConsumer setter) {
//         this.motorSpeed = motorSpeed;
//         this.setter = setter;
//         this.limitedSubsystem = LimitedSubsystem.UNLIMITED;
//     }

//     /**
//      * Periodically updates the percent output of a motor or group of
//      * motors and allows subsystem limits to be toggled on or off.
//      * 
//      * @param motorSpeed NetworkTables {@code GenericEntry} from which to get the speed.
//      * @param setter {@code DoubleConsumer} from a subsystem to use to set the motor speeds.
//      * @param climberSubsystem {@code ClimberSubsystem} to toggle limits of.
//      * @param limitEntry {@code GenericEntry} to get toggling boolean from.
//      */
//     public MotorUpdate(GenericEntry motorSpeed, DoubleConsumer setter,
//             ClimberSubsystem climberSubsystem, GenericEntry limitEntry) {
//         this.motorSpeed = motorSpeed;
//         this.setter = setter;
//         this.climberSubsystem = climberSubsystem;
//         this.toggleEntry = limitEntry;
//         this.limitedSubsystem = LimitedSubsystem.CLIMBER;
//     }

//     /**
//      * Periodically updates the percent output of a motor or group of
//      * motors and allows subsystem limits to be toggled on or off.
//      * 
//      * @param motorSpeed NetworkTables {@code GenericEntry} from which to get the speed.
//      * @param setter {@code DoubleConsumer} from a subsystem to use to set the motor speeds.
//      * @param targetingSubsystem {@code TargetingSubsystem} to toggle limits of.
//      * @param limitEntry {@code GenericEntry} to get toggling boolean from.
//      * @param function Function of {@code TargetingSubsystem} (angler or extension) whose limits to toggle.
//      */
//     public MotorUpdate(GenericEntry motorSpeed, DoubleConsumer setter,
//             TargetingSubsystem targetingSubsystem, GenericEntry limitEntry, TargetingFunction function) {
//         this.motorSpeed = motorSpeed;
//         this.setter = setter;
//         this.targetingSubsystem = targetingSubsystem;
//         this.targetingFunction = function;
//         this.toggleEntry = limitEntry;
//         this.limitedSubsystem = LimitedSubsystem.TARGETING;
//     }

//     /**
//      * Toggles the subsystem's motor limits.
//      * 
//      * @param climberSubsystem {@code ClimberSubsystem} whose limits to toggle.
//      * @param entry {@code GenericEntry} sending the limit boolean from Shuffleboard.
//      */
//     public void updateClimberLimits(ClimberSubsystem climberSubsystem, GenericEntry entry) {
//         while (climberSubsystem.getLimitsOff() == entry.getBoolean(false)) {
//             climberSubsystem.toggleLimits();
//         }
//     }

//     /**
//      * Toggles the subsystem's motor limits.
//      * 
//      * @param targetingSubsystem {@code TargetingSubsystem} whose angler limits to toggle.
//      * @param entry {@code GenericEntry} sending the limit boolean from Shuffleboard.
//      */
//     public void updateAnglerLimits(TargetingSubsystem targetingSubsystem, GenericEntry entry) {
//         while (targetingSubsystem.getAnglerLimitsOff() == entry.getBoolean(false)) {
//             targetingSubsystem.toggleAnglerLimits();
//         }
//     }

//     /**
//      * Toggles the subsystem's motor limits.
//      * 
//      * @param targetingSubsystem {@code TargetingSubsystem} whose extension limits to toggle.
//      * @param entry {@code GenericEntry} sending the limit boolean from Shuffleboard.
//      */
//     public void updateExtensionLimits(TargetingSubsystem targetingSubsystem, GenericEntry entry) {
//         while (targetingSubsystem.getExtensionLimitsOff() == entry.getBoolean(false)) {
//             targetingSubsystem.toggleExtensionLimits();
//         }
//     }
//     /**
//      * Calls the {@code DoubleConsumer} with the speed from the {@code GenericEntry} and updates limits (if any).
//      */
//     public void periodic() {
//         System.out.println("Entry reads " + motorSpeed.getDouble(Math.E));
//         setter.accept(motorSpeed.getDouble(0));
//         switch (limitedSubsystem) {
//             case CLIMBER:
//                 updateClimberLimits(climberSubsystem, toggleEntry);
//             break;

//             case TARGETING:
//                 switch (targetingFunction) {
//                     case ANGLER:
//                         updateAnglerLimits(targetingSubsystem, toggleEntry);
//                     break;

//                     case EXTENSION:
//                         updateExtensionLimits(targetingSubsystem, toggleEntry);
//                     break;
//                 }
//             break;

//             case UNLIMITED:

//             break;
//         }
//     }
// }