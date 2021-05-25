package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightManagerSubsystem;

public class GalacticSearch_Decision2_IsPathBRed extends CommandBase {

    private LimelightManagerSubsystem mLimelightManagerSubsystem;
    private boolean mIsFinished;

    public GalacticSearch_Decision2_IsPathBRed(LimelightManagerSubsystem limeLight) {

        mLimelightManagerSubsystem = limeLight;
        addRequirements(mLimelightManagerSubsystem);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        if (mLimelightManagerSubsystem.getIntakeLimelight().isTarget() && 
            Math.abs(mLimelightManagerSubsystem.getIntakeLimelight().getTx()) < 20.0) {
            // We see ball at B3, so commit to Path B Red.
            CommandScheduler.getInstance().schedule(new GalacticSearch_PathBRed());
        } else {
            // Move to next position to make next decision.
            CommandScheduler.getInstance().schedule(new GalacticSearch_AimForDecision3());
        }
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

}
