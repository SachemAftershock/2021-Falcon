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
        final double pow = ShapeStraight(mController.getDeadbandY(Hand.kLeft));
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

        // compute an exponential function that lowers close to zero, then rapidly increaseds close to 1
        // This should be more precise since it is the actual funtion the polynomial coeeficients were computer for, which are thus approxiations. 
        double theExponent = 4.0;  // range 1.0 to 10.0
        double theTargetMatchScale = 1.0;  // range 0.1 to 1.0

        if (Math.abs(rot) <  mController.getkJoystickDeadbandToleranceX())
          return 0.0;
        if (rot >= 0.0)
          return (Math.pow(Math.exp(rot-mController.getkJoystickDeadbandToleranceX()),theExponent)-1)/(Math.pow(Math.exp(1-mController.getkJoystickDeadbandToleranceX()),theExponent)-1)*theTargetMatchScale;
        else
          return -((Math.pow(Math.exp(-(rot-mController.getkJoystickDeadbandToleranceX())),theExponent)-1)/(Math.pow(Math.exp(1-mController.getkJoystickDeadbandToleranceX()),theExponent)-1))*theTargetMatchScale;
    }

    private double ShapeStraight(double pow){
      //Uses two sections of linear shaping for acceleration (explain better here later)
      double power = pow;
      
      double deadband = mController.getkJoystickDeadbandToleranceY();

      if(power > 0.0){
        if(power < deadband){
          //double deadband = 0.0;
          return 0.0;
        }
        else if(power < 0.70 && power > deadband){
          //double shapeTurnConstant = 0.50;
          return 0.70;
        }
        else{
          return 1.0;
        }
      }
      else if(power < 0.0){
        if(power > deadband){
          return 0.0;
        }
        else if(power < 0.70 && power > deadband){
          return -0.70;
        }
        else{
          return -1.0;
        }
      }
      return 0.0;
    }
    
  
}

