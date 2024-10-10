package frc.robot.sensors.Vision.MLVision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
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

    // ArrayList<Translation2d> noteMemory = new ArrayList<>();
    private Supplier<Pose2d> robotPos;
    

    public MLVision(MLVisionIO io, Supplier<Pose2d> robotPos) {
        this.io = io;
        this.robotPos = robotPos;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ML Vision", inputs);

        // Logger.recordOutput("MLVision/closest note", getClosestNote().isPresent() ? getClosestNote().get() : new Translation2d(-999, -999));

        // Logger.recordOutput("MLVision/Notes", );
        Logger.recordOutput("MLVision/NoteMemory", Arrays.stream(getFieldRelativeNotePos(robotPos.get()))
            .map((x) -> new Pose2d(x.pose, new Rotation2d())).toArray(Pose2d[]::new));
    }
    public boolean isTarget() {
        return inputs.isTarget;
    }

    public Note[] getCameraRelativeNotePos() {
        ArrayList<Note> posArray = new ArrayList<>();

        for (int n = 0; n < inputs.numVisibleNotes; n++) {
            double y = (MLConstants.noteHeight / 2 - MLConstants.cameraHeight);
            double x = (1 / Math.tan(Math.toRadians(inputs.allTy[n] + MLConstants.angle))) * y;
            double x1 = Math.cos(Math.toRadians(inputs.allTx[n])) * x;
            double y1 = Math.sin(Math.toRadians(inputs.allTx[n])) * x;
            Translation2d pos = new Translation2d(x1, -y1);
            if (MLConstants.FOV - Math.abs(inputs.allTx[n]) > MLConstants.FOVPadding + 2){
                posArray.add(new Note(pos, inputs.confidence[n]));
            }
        }
        return posArray.toArray(Note[]::new);
        // return sum.times(1/averageLength);
    }
    public Note[] getFieldRelativeNotePos(Pose2d robotPos){
        Translation2d[] poses = Arrays.stream(getCameraRelativeNotePos())
        .map((x) -> x.pose.rotateBy(new Rotation2d(robotPos.getRotation().getRadians() + Math.PI)).plus(robotPos.getTranslation()))
        .toArray(Translation2d[]::new);
        ArrayList<Note> notes = new ArrayList<>();
        for (int i = 0; i < getCameraRelativeNotePos().length; i++) {
            notes.add(new Note(poses[i], inputs.confidence[i]));
        }
        return notes.toArray(Note[]::new);
    }

    public Optional<Note> getClosestNote(Translation2d pose){

        Note[] notes = getFieldRelativeNotePos(robotPos.get());
        if (notes.length == 0){
            return Optional.empty();
        }
        Note best = new Note(new Translation2d(), 0);
        double bestDist = Double.POSITIVE_INFINITY;
        for (Note note : notes) {
            if (note.pose.getDistance(pose) < bestDist){
                best = note;
                bestDist = note.pose.getDistance(pose);
            }
        }
        return Optional.of(best);
    }
    public Translation2d getClosestNotePos(){
        if (getClosestNote(robotPos.get().getTranslation()).isPresent()){
            return getClosestNote(robotPos.get().getTranslation()).get().pose;
        }
        return new Translation2d();
    }
    public Note getClosestNote(){
        if (getClosestNote(robotPos.get().getTranslation()).isPresent()){
            return getClosestNote(robotPos.get().getTranslation()).get();
        }
        return new Note(new Translation2d(), -1);
    }

    public double getConfidence(){
        return getClosestNote().confidence;
    }

    // public void updateNoteMemory(Pose2d robotPos){
    //     // find notes which can be seen
    //     Translation2d[] canSee = getFieldRelativeNotePos(robotPos);
    //     Logger.recordOutput("MLVision/CanSee", canSee);
    //     ArrayList<Translation2d> validNotes = new ArrayList<>();
    //     for (Translation2d note : noteMemory) {
    //         double angle = Math.atan2(robotPos.getX() - note.getX(), robotPos.getY() - note.getY());
    //         if (angle >= -Math.toRadians(MLConstants.FOV/2 - MLConstants.FOVPadding) + robotPos.getRotation().getRadians() && 
    //                 angle <= Math.toRadians(MLConstants.FOV/2 - MLConstants.FOVPadding) + robotPos.getRotation().getRadians() && 
    //                 robotPos.getTranslation().getDistance(note) <= MLConstants.usableDistance){
    //             validNotes.add(note);
    //         }
    //     }
    //     Logger.recordOutput("MLVision/ValidNotes", validNotes.toArray(Translation2d[]::new));

    //     // clear notes which should be seen from memory, so actual seen notes can be readded.
        
    //     for (Translation2d note : validNotes) {
    //         noteMemory.remove(note);
    //     }

    //     // readd seen notes to memory

    //     for (Translation2d note : canSee) {
    //         if (note.getDistance(robotPos.getTranslation()) < MLConstants.usableDistance){
    //             noteMemory.add(note);
    //         }
    //     }
        
    // }
}