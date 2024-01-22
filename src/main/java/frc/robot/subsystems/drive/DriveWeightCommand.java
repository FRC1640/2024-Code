package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveWeights.DriveWeight;

public class DriveWeightCommand {

    static ArrayList<DriveWeight> weights = new ArrayList<>();
    Translation2d centerOfRot;
    ChassisSpeeds speeds;

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

    public static void removeWeight(String className) {
        Optional<Object> weight = Arrays.stream(weights.toArray())
                .filter(obj -> obj.getClass().getSimpleName().equals(className))
                .findFirst();
        if (weight.isPresent()){
            weights.remove(weight.get());
        }
    }

    public ChassisSpeeds getAllSpeeds() {
        speeds = new ChassisSpeeds();
        centerOfRot = new Translation2d();
        for (DriveWeight driveWeight : weights) {
            speeds = speeds.plus(driveWeight.getSpeeds());
            centerOfRot = centerOfRot.plus(driveWeight.getCenterOfRot());
            
        }

        
        return speeds;
    }
}
