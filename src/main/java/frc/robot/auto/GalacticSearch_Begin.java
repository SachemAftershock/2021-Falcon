package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightManagerSubsystem;
import frc.robot.subsystems.ConsoleSubsystem;


public class GalacticSearch_Begin extends SequentialCommandGroup {

    public GalacticSearch_Begin() {

        addCommands(
            
            // Robot is to start at C1 aimed eastward.

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "Galactic Search Begin"),

            new DelayCommand(DriveSubsystem.getInstance(), 1.0), 

            // Determine whether a ball is at the red position C2, then commit to red or blue path.
            new GalacticSearch_Decision1_IsPathARed(LimelightManagerSubsystem.getInstance())
        );
    }

}
