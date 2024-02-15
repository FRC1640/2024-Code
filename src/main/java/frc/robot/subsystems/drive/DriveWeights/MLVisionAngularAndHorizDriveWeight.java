package frc.robot.subsystems.drive.DriveWeights;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.sensors.Vision.MLVision;
import frc.robot.subsystems.drive.DriveWeightCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class MLVisionAngularAndHorizDriveWeight implements DriveWeight {

    private PIDController angularController = new PIDController(0.004, 0, 0); // Constants.PIDConstants.rotPID;
    private PIDController horizontalController = new PIDController(0.008, 0, 0); // Constants.PIDConstants.rotPID;


    private double angularVelocity;
    private double horizontalVelocity;
    private double verticalVelocity = 0.2; // ADD CONSTANT
    private MLVision vision;
    private Supplier<Rotation2d> angleSupplier;

    private CommandXboxController driveController;
    // private Supplier<Rotation2d> correctedAngleSupplier;

    private double deadband = 0; // 0.1;
    private double distanceLim = 10;
    private ChassisSpeeds chassisSpeedsToTurn = new ChassisSpeeds(0, 0, 0);

    private double initTime = 0;

    private boolean targetNoteSet = false; // target note signafies the note, out of the multiple visible, which is the objective to intake
    private double previousTX = 0;

    public MLVisionAngularAndHorizDriveWeight(MLVision vision, CommandXboxController driveController,
            Supplier<Rotation2d> angleSupplier) {
        this.vision = vision;
        this.angleSupplier = angleSupplier;
        this.driveController = driveController;

        scanForTargetNote();
        
        if (targetNoteSet){
            System.out.println("SET IN INITIALIZATION 1");
        }
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        angularVelocity = angularController.calculate(vision.getTX());
        angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;
        angularVelocity = MathUtil.clamp(angularVelocity, -1, 1);

        horizontalVelocity = horizontalController.calculate(vision.getTX());
        horizontalVelocity = (Math.abs(horizontalVelocity) < deadband) ? 0 : horizontalVelocity;
        horizontalVelocity = MathUtil.clamp(horizontalVelocity, -1, 1);


        if (!targetNoteSet){
            System.out.println("NO TARGET NOTE" + targetNoteSet + "Is a note visible? " + vision.isTarget());
            scanForTargetNote();
            chassisSpeedsToTurn = new ChassisSpeeds(0, 0, 0);
            return chassisSpeedsToTurn;
        }


        if (Math.abs(vision.getTX()) > distanceLim) {
                chassisSpeedsToTurn = new ChassisSpeeds(0, 0, angularVelocity);
                // return chassisSpeedsToTurn;
        } else if (!isDriveToNoteFinished()) {
                chassisSpeedsToTurn = ChassisSpeeds.fromRobotRelativeSpeeds(
                        new ChassisSpeeds(-verticalVelocity, -horizontalVelocity, 0),
                        angleSupplier.get());
                // return chassisSpeedsToTurn;
        }

        if (!vision.isTarget()) {
            System.out.println(" NO Target " + vision.isTarget() + targetNoteSet);

            scanForTargetNote();
            chassisSpeedsToTurn = new ChassisSpeeds(0, 0, 0);
            // return chassisSpeedsToTurn;
        }
        System.out.println("TEST PRINT PrevTX? " + targetNoteSet + "prev tx " + previousTX);
        return chassisSpeedsToTurn;
    }

    public boolean isDriveToNoteFinished() { // add intake limit later

        if (vision.isTarget()) {
            return false;
        } else if (!vision.isTarget()) {
            if (initTime == 0) {
                initTime = System.currentTimeMillis();
                return false;
            }

            if (initTime + 500 > System.currentTimeMillis()) {
                initTime = 0;
                return true;
            }

        }
        return false;
    }

    private void scanForTargetNote() {
        if (vision.isTarget()) {
            targetNoteSet = true;
            previousTX = vision.getTX();
        }
        System.out.println("SET to ---- " + targetNoteSet);

    }

    private double caclulatePredictedNextTX(){
        return 2.0;
    }

    // TO DO : Graph delta tx by rotational velocity to get an equation: with this suggested velocity (calculated from the current tx through the PID), how much will the tx change by? -> new tx


}