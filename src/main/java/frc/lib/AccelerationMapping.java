package frc.lib;

public class AccelerationMapping {

    private AftershockXboxController mController;

    public AccelerationMapping(){

    }

    public double linearShapeStraight(double pow){
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
            if(power < deadband) {return 0.0;}
            if(power <= stageOne && power > deadband) {return power*highPowerDampening;}
            if(power > stageOne && power <= stageTwo) {return power*mediumHighPowerDampening;}
            if(power > stageTwo && power <= stageThree) {return power*mediumPowerDampening;}
            if(power > stageThree || power <= stageFour) {return power*mediumLowPowerDampening;}
            if(power > stageFour || power <= stageFive) {return power*lowPowerDampening;}
            if(power > stageFive || power == stageSix) {return power*noPowerDampening;}
        }
          
        if(power < 0.0){
            if(power > -deadband) {return 0.0;}
            if(power >= -stageOne && power < -deadband) {return power*highPowerDampening;}
            if(power < -stageOne && power >= -stageTwo) {return power*mediumHighPowerDampening;}
            if(power < -stageTwo && power >= -stageThree) {return power*mediumPowerDampening; }
            if(power < -stageThree || power >= -stageFour) {return power*mediumLowPowerDampening;}
            if(power < -stageFour || power >= -stageFive) {return power*lowPowerDampening;}
            if(power < -stageFive || power == -stageSix) {return power*noPowerDampening;}
        }
        return 0;

    }


    public double linearShapeTurn(double rot){
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
            if(power < deadband) {return 0.0;}
            if(power <= stageOne && power > deadband) {return power*highPowerDampening;}
            if(power > stageOne && power <= stageTwo) {return power*mediumHighPowerDampening;}
            if(power > stageTwo && power <= stageThree) {return power*mediumPowerDampening;}
            if(power > stageThree || power <= stageFour) {return power*mediumLowPowerDampening;}
            if(power > stageFour || power <= stageFive) {return power*lowPowerDampening;}
            if(power > stageFive || power == stageSix) {return power*noPowerDampening;}
        }
  
        /*
            Figure out which values need to returned as negative, and wether to use < or >
            Going backwards might, and might not work because of this
        */
          
        if(power < 0.0){
            if(power > -deadband) {return 0.0;}
            if(power >= -stageOne && power < -deadband) {return power*highPowerDampening;}
            if(power < -stageOne && power >= -stageTwo) {return power*mediumHighPowerDampening;}
            if(power < -stageTwo && power >= -stageThree) {return power*mediumPowerDampening;}
            if(power < -stageThree || power >= -stageFour) {return power*mediumLowPowerDampening;}
            if(power < -stageFour || power >= -stageFive) {return -power*lowPowerDampening;}
            if(power < -stageFive || power == -stageSix) {return -power*noPowerDampening;}
        }
  
        return 0;
        
    }

}
