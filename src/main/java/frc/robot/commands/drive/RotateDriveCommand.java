package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RotateDriveCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mThetaSetpoint;
    
    //Field Relative Rotation, downfield is 0deg
    public RotateDriveCommand(DriveSubsystem drive, double thetaSetpoint) {
        mDrive = drive;
        mThetaSetpoint = thetaSetpoint;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.startAutoRotate(mThetaSetpoint);
        System.out.println("RotateDriveCommand started " + Double.toString(mThetaSetpoint) + " degrees.");
    }

    @Override
    public void execute() {
        mDrive.runAutoRotate();
    }

    @Override
    public boolean isFinished() {
        if (mDrive.rotateTargetReached()) {
            System.out.println("RotateDriveCommand completed " + Double.toString(mThetaSetpoint) + " degrees.");
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        mDrive.stop();
    }
}