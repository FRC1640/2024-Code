package frc.robot.sensors.Vision.MLVision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.periodic.PeriodicBase;
import frc.robot.Constants.MLConstants;

public class MLVision extends PeriodicBase {
    private MLVisionIO io;
    private MLVisionIOInputsAutoLogged inputs = new MLVisionIOInputsAutoLogged();

    ArrayList<Translation2d> noteMemory = new ArrayList<>();
    private Supplier<Pose2d> robotPos;
    

    public MLVision(MLVisionIO io, Supplier<Pose2d> robotPos) {
        this.io = io;
        this.robotPos = robotPos;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ML Vision", inputs);

        updateNoteMemory(robotPos.get());
        Logger.recordOutput("MLVision/NoteMemory", noteMemory.toArray(Translation2d[]::new));
    }

    // Getters

    public double getLatency() {
        return inputs.latency;
    }

    public boolean isTarget() {
        return inputs.isTarget;
    }

    public double getTX() {
        if (inputs.isTargetNote) {
            return inputs.calculatedTx;
        } else {
            return inputs.tx;
        }
    }

    public double getTY() {
        if (inputs.isTargetNote) {
            return inputs.calculatedTy;
        } else {
            return inputs.ty;
        }
    }

    public double getTA() {
        if (inputs.isTargetNote) {
            return inputs.calculatedTa;
        } else {
            return inputs.ta;
        }
    }

    public Command waitUntilMLCommand(double taMin, double txMax) {
        // return new WaitUntilCommand(()->(getTA() > ta) && (getTX() > tx));
        // return new WaitUntilCommand
        return new WaitUntilCommand(() -> isTarget() && getTA() > taMin);

    }

    public Translation2d[] getCameraRelativeNotePos() {
        ArrayList<Translation2d> posArray = new ArrayList<>();

        for (int n = 0; n < inputs.numVisibleNotes; n++) {
            double y = (MLConstants.noteHeight / 2 - MLConstants.cameraHeight);
            double x = (1 / Math.tan(Math.toRadians(inputs.allTy[n] + MLConstants.angle))) * y;
            double x1 = Math.cos(Math.toRadians(inputs.allTx[n])) * x;
            double y1 = Math.sin(Math.toRadians(inputs.allTx[n])) * x;

            Translation2d pos = new Translation2d(x1, -y1);
            posArray.add(pos);
        }
        return (Translation2d[])posArray.toArray();
        // return sum.times(1/averageLength);
    }
    public Translation2d[] getFieldRelativeNotePos(Pose2d robotPos){
        return Arrays.stream(getCameraRelativeNotePos())
        .map(x -> x.rotateBy(new Rotation2d(robotPos.getRotation().getRadians() + Math.PI)).plus(robotPos.getTranslation()))
        .toArray(Translation2d[]::new);
    }

    public void updateNoteMemory(Pose2d robotPos){
        // find notes which can be seen
        Translation2d[] canSee = getFieldRelativeNotePos(robotPos);
        Logger.recordOutput("MLVision/CanSee", canSee);
        ArrayList<Translation2d> validNotes = new ArrayList<>();
        for (Translation2d note : noteMemory) {
            double angle = Math.atan2(robotPos.getX() - note.getX(), robotPos.getY() - note.getY());
            if (angle >= -Math.toRadians(MLConstants.FOV/2 - MLConstants.FOVPadding) + robotPos.getRotation().getRadians() && 
                    angle <= Math.toRadians(MLConstants.FOV/2 - MLConstants.FOVPadding) + robotPos.getRotation().getRadians() && 
                    robotPos.getTranslation().getDistance(note) <= MLConstants.usableDistance){
                validNotes.add(note);
            }
        }
        Logger.recordOutput("MLVision/ValidNotes", validNotes.toArray(Translation2d[]::new));

        // clear notes which should be seen from memory, so actual seen notes can be readded.
        
        for (Translation2d note : validNotes) {
            noteMemory.remove(note);
        }

        // readd seen notes to memory

        for (Translation2d note : canSee) {
            if (note.getDistance(robotPos.getTranslation()) < MLConstants.usableDistance){
                noteMemory.add(note);
            }
        }
        
    }
}