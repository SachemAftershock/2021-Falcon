package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightManagerSubsystem;
import frc.robot.subsystems.ConsoleSubsystem;


public class GalacticSearch_AimForDecision2 extends SequentialCommandGroup {

        // arctan(opposite/adjacent), (opposite = delta letter, adjacent = delta number)
        // hypootneus = adjacent / cos(theta)

        final double AngleC1B3 = -(Math.toDegrees(Math.atan2(-1.0, 2.0))); 

    public GalacticSearch_AimForDecision2() {

        addCommands(
            
            // Should be at C1+21"

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "Was not Path A Red, Prep to decide whether Path B Red"),
            
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleC1B3), 
            //new DelayCommand(DriveSubsystem.getInstance(), 2.0),

            // Determine whether a ball is at the red position C2, then commit to red or blue path.
            new GalacticSearch_Decision2_IsPathBRed(LimelightManagerSubsystem.getInstance())
        );
    }

}
