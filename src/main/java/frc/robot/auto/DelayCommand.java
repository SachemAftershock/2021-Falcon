package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;

public class DelayCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mSecondsToDelay;
    private Timer mTimer;

    public DelayCommand(DriveSubsystem drive, double theSecondsToDelay) {
        mDrive = drive;
        mSecondsToDelay = theSecondsToDelay;
        mTimer = new Timer();
        addRequirements(mDrive);
    }   

    @Override
    public void initialize() {
        mDrive.startAutoDrive(mSecondsToDelay);
        System.out.println("DelayCommand started " + Double.toString(mSecondsToDelay) + " seconds.");
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        
        if (mTimer.hasElapsed(mSecondsToDelay)) {
            System.out.println("DelayCommand completed " + Double.toString(mSecondsToDelay) + " seconds.");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mTimer.stop();
    }
}