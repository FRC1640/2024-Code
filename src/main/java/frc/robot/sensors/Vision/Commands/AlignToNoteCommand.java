package frc.robot.sensors.Vision.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.sensors.Vision.MLVision;
//import frc.robot.subsystems.drive.DriveSubsystem;

public class AlignToNoteCommand extends Command{
    //private DriveSubsystem driveSubsystem;
    private MLVision MLVision;

    private PIDController horizontalPIDController;
    private PIDController verticalPIDController;
    private PIDController angController;

    private double horizontalVelocity;
    private double verticalVelocity;
    private double angularVelocity;
    private double desiredDistanceToNote;
    private double deadband;
    private double kP;
    private double isFinishedTolerance;


    public AlignToNoteCommand( MLVision MLVision) {
        //this.driveSubsystem = driveSubsystem;
        this.MLVision = MLVision;

        //addRequirements(driveSubsystem);
    }


    /**
    * Initialize the command and create PID controllers for horizontal, vertical,
    * and angular movements.
    */
    public void initialize() {
        horizontalPIDController = new PIDController(kP, 0, 0);
        verticalPIDController = new PIDController(kP, 0, 0);
        angController = new PIDController(kP, 0, 0);
    }

    /**
    * Execute the command. Calculate velocities using PID controllers based on the
    * Limelight data, apply deadbands,
    * and drive the drivetrain.
    */
    public void execute() {
        //if (fieldRelative) {
            horizontalVelocity = horizontalPIDController.calculate(MLVision.getTX());
        //} else {
        //    m_angularVelocity = m_angController.calculate(m_limelightSubsystem.getHorizontalTargetAngle());
            angularVelocity = angController.calculate(MLVision.getTX());
        //}

        verticalVelocity = verticalPIDController.calculate((MLVision.getDistance() - desiredDistanceToNote) * 100);

        verticalVelocity = (Math.abs(verticalVelocity) < deadband) ? 0 : verticalVelocity;
        horizontalVelocity = (Math.abs(horizontalVelocity) < deadband) ? 0 : horizontalVelocity;
        angularVelocity = (Math.abs(angularVelocity) < deadband) ? 0 : angularVelocity;

       // m_drivetrainSubsystem.drive(
               // m_verticalVelocity,
               // m_horizontalVelocity,
                //m_angularVelocity,);
        
        //driveSubsystem.drivePercentDoubleCone(0, verticalVelocity, angularVelocity, false);


        }

    /**
     * @return True if the command is considered finished (when the robot is within
     *         a certain tolerance of the target),
     *         false otherwise.
     */

    public boolean isFinished() {
        if (Math.abs(MLVision.getTX()) > isFinishedTolerance // dont get this...
                || (Math.abs(MLVision.getDistance() - desiredDistanceToNote) > isFinishedTolerance / 5)) {
            return false;
        }
        return true;
    }

    /**
     * @param interrupted If the command was interrupted or not
     *                    Stops the drivetrain when the command ends, regardless of
     *                    reason (successful completion or interruption).
     */
    @Override
    public void end(boolean interrupted) {
        //driveSubsystem.drive(0, 0, 0, true);
    }

}
