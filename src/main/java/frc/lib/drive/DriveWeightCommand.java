package frc.lib.drive;

import java.util.ArrayList;
import java.util.Iterator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class DriveWeightCommand {
    static ArrayList<DriveWeight> persistentWeights = new ArrayList<>();

    static ArrayList<DriveWeight> weights = new ArrayList<>();
    Translation2d centerOfRot = new Translation2d();
    static ChassisSpeeds speeds = new ChassisSpeeds();
    static double setAngle = 0;

    public Command create(DriveSubsystem driveSubsystem) {
        Command c = Commands.race(new RunCommand(() ->getAllSpeeds()),
                driveSubsystem.driveDoubleConeCommand(() -> speeds, () -> centerOfRot));
        c.addRequirements(driveSubsystem);
        return c;
    }

    public static void addWeight(DriveWeight weight) {
        if (!weights.contains(weight)) {
            weights.add(weight);
        }
    }

    public static void removeWeight(DriveWeight weight) {
        if (weights.contains(weight)) {
            weights.remove(weight);
        }
    }

    public static double getAngle(){
        return setAngle;
    }

    public static void addPersistentWeight(DriveWeight weight){
        if (!persistentWeights.contains(weight)) {
            persistentWeights.add(weight);
        }
    }

    public static void removePersistentWeight(DriveWeight weight) {
        if (persistentWeights.contains(weight)) {
            persistentWeights.remove(weight);
        }
    }

    public static void removeAllWeights(){
        weights.clear();
    }

    public static ChassisSpeeds getSpeeds(){
        return speeds;
    }

    public void getAllSpeeds() {
        speeds = new ChassisSpeeds();
        centerOfRot = new Translation2d();
        setAngle = 0;
        // remove weights with a true cancel condition
        Iterator<DriveWeight> iterator = weights.iterator();
        while (iterator.hasNext()) {
            DriveWeight weight = iterator.next();
            if (weight.cancelCondition()){
                iterator.remove();
            }
        }
        // iterate over remaining weights and add speeds
        for (DriveWeight driveWeight : weights) {
            speeds = speeds.plus(driveWeight.getSpeeds().times(driveWeight.getWeight()));
            centerOfRot = centerOfRot.plus(driveWeight.getCenterOfRot());
            setAngle += driveWeight.angle();
        }
        for (DriveWeight driveWeight : persistentWeights) {
            speeds = speeds.plus(driveWeight.getSpeeds().times(driveWeight.getWeight()));
            centerOfRot = centerOfRot.plus(driveWeight.getCenterOfRot());
            setAngle += driveWeight.angle();
        }
        centerOfRot.times(1/(weights.size() + persistentWeights.size()));
        speeds = decreaseSpeeds(speeds);
    }

    public static int getWeightsSize(){
        return weights.size() + persistentWeights.size();
    }
    
    public ChassisSpeeds decreaseSpeeds(ChassisSpeeds speeds){
        double max = Math.max(Math.hypot(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond),speeds.omegaRadiansPerSecond);
        if (max > 1){
            speeds = speeds.times(1/max);
            // System.out.println(speeds);
        }
        
        return speeds;
    }
}
