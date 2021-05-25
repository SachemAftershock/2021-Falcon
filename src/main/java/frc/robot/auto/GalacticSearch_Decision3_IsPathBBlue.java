package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightManagerSubsystem;

public class GalacticSearch_Decision3_IsPathBBlue extends CommandBase {

    private LimelightManagerSubsystem mLimelightManagerSubsystem;
    private boolean mIsFinished;

    public GalacticSearch_Decision3_IsPathBBlue(LimelightManagerSubsystem limeLight) {

        mLimelightManagerSubsystem = limeLight;
        addRequirements(mLimelightManagerSubsystem);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        if (mLimelightManagerSubsystem.getIntakeLimelight().isTarget() && 
            Math.abs(mLimelightManagerSubsystem.getIntakeLimelight().getTx()) < 10.0) {
            CommandScheduler.getInstance().schedule(new GalacticSearch_PathBBlue());
        } else {
            CommandScheduler.getInstance().schedule(new GalacticSearch_PathABlue());
        }
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

}
