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
      /*
        Uses a linear system to apply a multiplyer to the inputed throttle in order to 
        dampen rapid changes in accleration
      */

        double power = rot;
        double deadband = mController.getkJoystickDeadbandToleranceX();
        
        /*
          4 levels of power dampening applied based on inputed throttle
          The values are not final and can and should be tuned to meet the requirements of the robot
        */
        double highPowerDampening = 0.15;//0.25;
        double mediumHighPowerDampening = 0.20;
        double mediumPowerDampening = 0.25;//0.50;
        double mediumLowPowerDampening = 0.30;
        double lowPowerDampening = 0.35;//0.75;
        double noPowerDampening = 0.40;//1.0;

        /*
          Ranges for inputed throttle
          Can be tuned in conjunction to the power dampening levels to create smooth acceleration
        */
        double stageOne = 0.30;
        double stageTwo = 0.50;
        double stageThree = 0.70;
        double stageFour = 0.80;
        double stageFive = 0.90;
        double stageSix = 1.0;

        /*
          Up to 50 percent throttle has high power dampening applied to it, meaning the motors are only 
          commanded to 25 percent of the throttle inputed

          Up to 80 percent throttle has medium power dampening applied to it, meaning the motors are only
          commanded to 50 percent of the throttle inputed

          Up to 90 percent throttle has low power dampening applied to it, meaning the motors are commanded
          to 75 percent of the inputed throttle

          When 90 percent to 100 percent throttle is applied there is no power dampening

          The deadband of up to 15 percent is accounted for
        */

        if(power > 0.0){
          if(power < deadband) {
            return 0.0;
          }
          if(power <= stageOne && power > deadband) {
            return power*highPowerDampening;
          }
          if(power > stageOne && power <= stageTwo) {
            return power*mediumHighPowerDampening;
          }
          if(power > stageTwo && power <= stageThree) {
            return power*mediumPowerDampening; 
          }
          if(power > stageThree || power <= stageFour) {
            return power*mediumLowPowerDampening;
          }
          if(power > stageFour || power <= stageFive) {
            return power*lowPowerDampening;
          }
          if(power > stageFive || power == stageSix) {
            return power*noPowerDampening;
          }
        }

        /*
          Figure out which values need to returned as negative, and wether to use < or >
          Going backwards might, and might not work because of this
        */
        
        if(power < 0.0){
          if(power > -deadband) {
            return 0.0;
          }
          if(power >= -stageOne && power < -deadband) {
            return power*highPowerDampening;
          }
          if(power < -stageOne && power >= -stageTwo) {
            return power*mediumHighPowerDampening;
          }
          if(power < -stageTwo && power >= -stageThree) {
            return power*mediumPowerDampening; 
          }
          if(power < -stageThree || power >= -stageFour) {
            return power*mediumLowPowerDampening;
          }
          if(power < -stageFour || power >= -stageFive) {
            return -power*lowPowerDampening;
          }
          if(power < -stageFive || power == -stageSix) {
            return -power*noPowerDampening;
          }
        }

        return 0;
        
    }

    private double ShapeStraight(double pow){

      /*
        Uses a linear system to apply a multiplyer to the inputed throttle in order to 
        dampen rapid changes in accleration
      */

        double power = pow;
        double deadband = mController.getkJoystickDeadbandToleranceY();
        
        /*
          4 levels of power dampening applied based on inputed throttle
          The values are not final and can and should be tuned to meet the requirements of the robot
        */
        double highPowerDampening = 0.15;//0.25;
        double mediumHighPowerDampening = 0.20;
        double mediumPowerDampening = 0.25;//0.50;
        double mediumLowPowerDampening = 0.30;
        double lowPowerDampening = 0.35;//0.75;
        double noPowerDampening = 0.40;//1.0;

        /*
          Ranges for inputed throttle
          Can be tuned in conjunction to the power dampening levels to create smooth acceleration
        */
        double stageOne = 0.30;
        double stageTwo = 0.50;
        double stageThree = 0.70;
        double stageFour = 0.80;
        double stageFive = 0.90;
        double stageSix = 1.0;

        /*
          Up to 50 percent throttle has high power dampening applied to it, meaning the motors are only 
          commanded to 25 percent of the throttle inputed

          Up to 80 percent throttle has medium power dampening applied to it, meaning the motors are only
          commanded to 50 percent of the throttle inputed

          Up to 90 percent throttle has low power dampening applied to it, meaning the motors are commanded
          to 75 percent of the inputed throttle

          When 90 percent to 100 percent throttle is applied there is no power dampening

          The deadband of up to 15 percent is accounted for
        */

        if(power > 0.0){
          if(power < deadband) {
            return 0.0;
          }
          if(power <= stageOne && power > deadband) {
            return power*highPowerDampening;
          }
          if(power > stageOne && power <= stageTwo) {
            return power*mediumHighPowerDampening;
          }
          if(power > stageTwo && power <= stageThree) {
            return power*mediumPowerDampening; 
          }
          if(power > stageThree || power <= stageFour) {
            return power*mediumLowPowerDampening;
          }
          if(power > stageFour || power <= stageFive) {
            return power*lowPowerDampening;
          }
          if(power > stageFive || power == stageSix) {
            return power*noPowerDampening;
          }
        }

        /*
          Figure out which values need to returned as negative, and wether to use < or >
          Going backwards might, and might not work because of this
        */
        
        if(power < 0.0){
          if(power > -deadband) {
            return 0.0;
          }
          if(power >= -stageOne && power < -deadband) {
            return power*highPowerDampening;
          }
          if(power < -stageOne && power >= -stageTwo) {
            return power*mediumHighPowerDampening;
          }
          if(power < -stageTwo && power >= -stageThree) {
            return power*mediumPowerDampening; 
          }
          if(power < -stageThree || power >= -stageFour) {
            return power*mediumLowPowerDampening;
          }
          if(power < -stageFour || power >= -stageFive) {
            return power*lowPowerDampening;
          }
          if(power < -stageFive || power == -stageSix) {
            return power*noPowerDampening;
          }
        }

        return 0;
    }
}

