package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.AftershockXboxController;
import frc.robot.subsystems.DriveSubsystem;

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
    
    @Override
    public void execute() {
        final double pow = mController.getDeadbandY(Hand.kLeft);
        final double rot = ShapeTurn(mController.getDeadbandX(Hand.kRight));
        final boolean leftTriggerPressed = mController.getTriggerHeld(Hand.kLeft);
        final boolean rightTriggerPressed = mController.getTriggerHeld(Hand.kRight);
        final boolean wantDeccelerate = leftTriggerPressed || rightTriggerPressed;
        mDrive.manualDrive(pow, rot, wantDeccelerate);
    }

    /**
     * shapeTurn method.  The manual robot turning is too uncontrollable, 
     * This method makes the turning slower when stick is nearer the idle position,
     * then speeds up non-linearly as approach maximum stick deflections.
     * The functions (or coefficients) were computed using Excel spreadsheet.
     */
    private double ShapeTurn(double rot){

        if(false) 
        {
            // apply a polynomial regression, sythesized from an Excel spreadsheet
            double C4 = 1.541629254, C3 = -1.463434215, C2 = 0.805405022, C1 = -0.026471167, C0 = 0.141308946; 
            if (rot >= 0.0)
              return C4*Math.pow(rot,4) + C3*Math.pow(rot,3) + C2*Math.pow(rot,2) + C1*rot + C0;
            else
              return -(C4*Math.pow(-rot,4) + C3*Math.pow(-rot,3) + C2*Math.pow(-rot,2) + C1*(-rot) + C0);
        } else {
            // compute an exponential function that lowers close to zero, then rapidly increaseds close to 1
            // This should be more precise since it is the actual funtion the polynomial coeeficients were computer for, which are thus approxiations. 
            double theExponent = 4;
            double theDeadbandOffset = 0.15;
            if (rot >= 0.0)
              return (Math.pow(Math.exp(rot),theExponent)-1)/(Math.pow(Math.exp(1),theExponent)-1)*(1-theDeadbandOffset)+theDeadbandOffset;
            else
              return -((Math.pow(Math.exp(-rot),theExponent)-1)/(Math.pow(Math.exp(1),theExponent)-1)*(1-theDeadbandOffset)+theDeadbandOffset);
        }
    }

}
