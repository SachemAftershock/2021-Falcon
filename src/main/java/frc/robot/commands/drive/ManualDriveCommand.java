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
    AccelerationMapping accelerationLimiter;

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

    
    @Override
    public void execute() {
        boolean secondaryDriveControlls = mController.getYButtonPressed(); 
        //System.out.println(secondaryDriveControlls);
        if(secondaryDriveControlls){
            final double forward = AccelerationMapping.linearShapeStraight(mController.getTriggerAxis(Hand.kRight), mController.getkJoystickDeadbandToleranceY());
            final double backward = AccelerationMapping.linearShapeStraight(-(mController.getTriggerAxis(Hand.kLeft)), mController.getkJoystickDeadbandToleranceY());
            final double pow = forward + backward;
            final double rot = AccelerationMapping.linearShapeTurn(mController.getDeadbandX(Hand.kLeft), mController.getkJoystickDeadbandToleranceX());
            mDrive.manualDrive(pow, rot, false /*wantDeccelerate*/);
        }
        
        if(!secondaryDriveControlls){
            final double pow = AccelerationMapping.linearShapeStraight(mController.getDeadbandY(Hand.kLeft), mController.getkJoystickDeadbandToleranceY());
            final double rot = AccelerationMapping.linearShapeTurn(mController.getDeadbandX(Hand.kRight), mController.getkJoystickDeadbandToleranceX());
            final boolean leftTriggerPressed = mController.getTriggerHeld(Hand.kLeft);
            final boolean rightTriggerPressed = mController.getTriggerHeld(Hand.kRight);
            final boolean wantDeccelerate = leftTriggerPressed || rightTriggerPressed;
            mDrive.manualDrive(pow, rot, wantDeccelerate);
        }

    } 
}

