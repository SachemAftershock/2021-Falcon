package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.AftershockXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.lib.AccelerationMapping;

/**
 * Default Manual Operator Drive Command
 * 
 * @author Shreyas Prasad
 */
public class ManualDriveCommand extends CommandBase {
 
    private DriveSubsystem mDrive;
    private AftershockXboxController mController;

    /**
     * Constructor for ManualDriveCommand Class
     * 
     * @param drive DriveSubsystem singleton instance
     * 
     * @param controller Primary Xbox Controller
     */
    public ManualDriveCommand(DriveSubsystem drive, AftershockXboxController controller) {
        mDrive = drive;
        mController = controller;
        addRequirements(mDrive);
    }

    AccelerationMapping accelerationLimiter = new AccelerationMapping();
    
    @Override
    public void execute() {
        final double pow = accelerationLimiter.linearShapeStraight(mController.getDeadbandY(Hand.kLeft));
        final double rot = accelerationLimiter.linearShapeTurn(mController.getDeadbandX(Hand.kRight));
        final boolean leftTriggerPressed = mController.getTriggerHeld(Hand.kLeft);
        final boolean rightTriggerPressed = mController.getTriggerHeld(Hand.kRight);
        final boolean wantDeccelerate = leftTriggerPressed || rightTriggerPressed;
        mDrive.manualDrive(pow, rot, wantDeccelerate);
        
    } 
}

