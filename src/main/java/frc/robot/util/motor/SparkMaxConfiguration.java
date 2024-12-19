// package frc.robot.util.motor;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.OptionalInt;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// public class SparkMaxConfiguration {

    /*
     * TODO features to transfer:
     * idle mode √
     * inversion √
     * smart current limit √
     * encoder measurement period √
     * encoder average depth √
     * can timeout
     * limit switches (through configuration? what about limit switch stuff in status frames?) 
     * status frames √
     * burn flash...
     */

//     private IdleMode idleMode;
//     private boolean inverted;
//     private int smartCurrentLimit;
//     private int encoderMeasurementPeriod;
//     private int encoderAverageDepth;
//     private OptionalInt canTimeout;
//     private List<LimitSwitchConfiguration> limitSwitches;
//     private StatusFrames statusFrames;
//     private boolean burn = false;    

//     public SparkMaxConfiguration(IdleMode idleMode, boolean inverted, int smartCurrentLimit,
//             int encoderMeasurementPeriod, int encoderAverageDepth, OptionalInt canTimeout, StatusFrames statusFrames) {
//         this.idleMode = idleMode;
//         this.inverted = inverted;
//         this.smartCurrentLimit = smartCurrentLimit;
//         this.encoderMeasurementPeriod = encoderMeasurementPeriod;
//         this.encoderAverageDepth = encoderAverageDepth;
//         this.canTimeout = canTimeout;
//         this.limitSwitches = new ArrayList<>();
//         this.statusFrames = statusFrames;
//     }

//     public SparkMaxConfiguration(IdleMode idleMode, boolean inverted, int smartCurrentLimit, int encoderMeasurementPeriod,
//             int encoderAverageDepth, OptionalInt canTimeout, StatusFrames statusFrames, LimitSwitchConfiguration ... limitSwitches) {
//         this.idleMode = idleMode;
//         this.inverted = inverted;
//         this.smartCurrentLimit = smartCurrentLimit;
//         this.encoderMeasurementPeriod = encoderMeasurementPeriod;
//         this.encoderAverageDepth = encoderAverageDepth;
//         this.canTimeout = canTimeout;
//         this.limitSwitches = new ArrayList<>();
//         for (LimitSwitchConfiguration switchConfig : limitSwitches) {
//             this.limitSwitches.add(switchConfig);
//         }
//         this.statusFrames = statusFrames;
//     }

//     private void configIdleMode(SparkMax spark) {
//         if (spark.getIdleMode() != idleMode) {
//             spark.setIdleMode(idleMode);
//             burn = true;
//         }
//     }

//     private void configInverted(SparkMax spark) {
//         if (spark.getInverted() != inverted) {
//             spark.setInverted(inverted);
//             burn = true;
//         }
//     }

//     private void configSmartCurrentLimit(SparkMax spark) {
//         spark.setSmartCurrentLimit(smartCurrentLimit);
//     }

//     private void configEncoderMeasurementPeriod(SparkMax spark) {
//         spark.getEncoder().setMeasurementPeriod(encoderMeasurementPeriod);
//     }

//     private void configEncoderAverageDepth(SparkMax spark) {
//         if (spark.getEncoder().getAverageDepth() != encoderAverageDepth) {
//             spark.getEncoder().setAverageDepth(encoderAverageDepth);
//             burn = true;
//         }
//     }

//     private void configCANTimeout(SparkMax spark) {
//         if (canTimeout.isPresent()) {
//             spark.setCANTimeout(canTimeout.getAsInt());
//         }
//     }

//     private void configLimitSwitches(SparkMax spark) {
//         for (LimitSwitchConfiguration switchConfig : limitSwitches) {
//             if (switchConfig.differentFrom(spark)) {
//                 switchConfig.apply(spark);
//                 burn = true;
//             }
//         }
//     }

//     private void updateStatusFrames(SparkMax spark) {
//         statusFrames.updateStatusFrames(spark);
//     }

//     private void burnFlash(SparkMax spark) {
//         spark.burnFlash();
//     }

//     public boolean getFlashed() {
//         return burn;
//     }

//     public void config(SparkMax spark) {
//         configIdleMode(spark);
//         configInverted(spark);
//         configSmartCurrentLimit(spark);
//         configEncoderMeasurementPeriod(spark);
//         configEncoderAverageDepth(spark);
//         configCANTimeout(spark);
//         configLimitSwitches(spark);
//         updateStatusFrames(spark);
//         if (burn) { burnFlash(spark); }
//     }
// }
