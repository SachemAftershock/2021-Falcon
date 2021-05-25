package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightManagerSubsystem;

public class GalacticSearch_Decision1_IsPathARed extends CommandBase {

    private LimelightManagerSubsystem mLimelightManagerSubsystem;
    private boolean mIsFinished;

    public GalacticSearch_Decision1_IsPathARed(LimelightManagerSubsystem limeLight) {

        mLimelightManagerSubsystem = limeLight;
        addRequirements(mLimelightManagerSubsystem);
        mIsFinished = false;
    }

    @Override
    public void initialize() {
        if (mLimelightManagerSubsystem.getIntakeLimelight().isTarget() && 
            Math.abs(mLimelightManagerSubsystem.getIntakeLimelight().getTx()) < 25.0) {
            // We are at C1, we saw a ball immediatly in front of us at C3, so commit to Path A Red.
            CommandScheduler.getInstance().schedule(new GalacticSearch_PathARed());
        } else {
            // Move to next position to make next decision.
            CommandScheduler.getInstance().schedule(new GalacticSearch_AimForDecision2());
        }
        mIsFinished = true;
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

}
