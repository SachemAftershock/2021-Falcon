package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ConsoleSubsystem;

public class AutoNavChallengeBouncePath extends SequentialCommandGroup {

    public AutoNavChallengeBouncePath () {

        addCommands(

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "Started Bounce Path"),

            // Robot starts at C1, facing eastward, relative to Figure 2-5.

            // Get to Waypoint A3
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kEastward - 7),            // Should already start aimed this way.
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2),  // C1 to C3
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kNorthward - 10),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2),  // C3 to A3 

            // Back out to waypoint E5
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * -2), // A3 to C3 
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kNorthward + 45.0 - 5.0),    // Robot aimed North-West to continue driving backwards
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kDiag45Inches * -2 + 5),   // C3 to E5             

            // Drive forward to Waypoint A6
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kNorthward - 45.0 + 5.0),    // Robot aimed North-East to drive forwad next.
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kDiag45Inches * 1),    // E5 to D6
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kNorthward),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 3),  // D6 to A6 

            // Back out to waypoint E6
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * -4), // A6 to E6 

            // Get to Waypoint A9
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kEastward), 
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 3 - 7),  // E6 to E9
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kNorthward - 5.0),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 4),  // E9 to A9 

            // Back into finish zone!
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * -2), // A9 to C9 
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kEastward),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2),  // C9 to C11          

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "Finished Bounce Path")
        );
    }

}
