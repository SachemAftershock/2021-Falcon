package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


/**
 * Command to automatically drive forward linearly a defined distance
 * 
 * @author Shreyas Prasad
 */
public class LinearDriveCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mSetpoint;

    /**
     * Constructor for LinearDriveCommand Class
     * 
     * @param drive DriveSubsystem singleton instance
     * 
     * @param setpointInches distance to travel linearly in inches
     */
    public LinearDriveCommand(DriveSubsystem drive, double setpointInches) {
        mDrive = drive;
        mSetpoint = setpointInches;
        addRequirements(mDrive);
    }   

    @Override
    public void initialize() {
        mDrive.startAutoDrive(mSetpoint);
        System.out.println("LinearDriveCommand started " + Double.toString(mSetpoint) + " inches.");
    }

    @Override
    public void execute() {
        mDrive.runAutoDrive();
    }

    @Override
    public boolean isFinished() {
        
        if (mDrive.isLinearDriveTargetReached()) {
            System.out.println("LinearDriveCommand completed " + Double.toString(mSetpoint) + " inches.");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }
}