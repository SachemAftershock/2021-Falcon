package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightManagerSubsystem;
import frc.robot.subsystems.ConsoleSubsystem;


public class GalacticSearch_AimForDecision3 extends SequentialCommandGroup {

    public GalacticSearch_AimForDecision3() {

        addCommands(
            
            // Should be at C1, aimed at B3.

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "Was not Path B Red, Prep to decide whether Path B Blue"),
            
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kEastward),  // aim downfield

            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 4), // C1 to C5

            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kEastward + 45),  // aim at D6

            // Determine whether a ball is at the red position C2, then commit to red or blue path.
            new GalacticSearch_Decision3_IsPathBBlue(LimelightManagerSubsystem.getInstance())
        );
    }

}
