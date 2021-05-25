package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ConsoleSubsystem;

public class AutoNavChallengeSlalomPath extends SequentialCommandGroup {

    public AutoNavChallengeSlalomPath () {

        addCommands(

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "Started Slalom Path"),

            // Robot starts at E1, facing eastward, relative to Figure 2-6.

            // Drive to position E3
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 2),   // Drive from E1 to E3
           
            // Swoop up to 'above the line'
            new RotateDriveCommand(DriveSubsystem.getInstance(), 90),       // Turn CCW to face northward
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 2),   // Drive from E3 to C3
            new RotateDriveCommand(DriveSubsystem.getInstance(), 7),        // Turn CW to face eastward

            // Shoot down the long straight 'above the line'
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 6),   // Drive from C3 to C9

            // Swoop down to 'below the line'
            new RotateDriveCommand(DriveSubsystem.getInstance(), -90),      // Turn CW to face southward
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 2 - 5),//NOT TESTED experimental -10   // Drive from C9 to E9
            new RotateDriveCommand(DriveSubsystem.getInstance(), 0),        // Turn CCW to face easthward

            // Circle D10
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 1.7),   // Drive from E9 to E11
            new RotateDriveCommand(DriveSubsystem.getInstance(), 90),       // Turn CCW to face northward
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 2),   // Drive from E11 to C11
            new RotateDriveCommand(DriveSubsystem.getInstance(), 180),      // Turn CCW to face westward
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 1.4 + 10), // Drive from C11 to C9
            new RotateDriveCommand(DriveSubsystem.getInstance(), -90),      // Turn CCW to face southward
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 2 -5),   // Drive from C9 to E9
            new RotateDriveCommand(DriveSubsystem.getInstance(), -170),       // Turn CW to face westward

            // Shoot down the long straight 'below the line'
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 5.5 + 6),  // Drive from E9 to E3

            // Swoop up to 'above the line'
            new RotateDriveCommand(DriveSubsystem.getInstance(), 90),        // Turn CW to face northward
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 1.2 + 6),  // Drive from E3 to C3
            new RotateDriveCommand(DriveSubsystem.getInstance(), 180),       // Turn CCW to face westward

            // Race into the finish zone! C1
            new LinearDriveCommand(DriveSubsystem.getInstance(), 30 * 2),     // Drive from C3 to C1
            
            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "Finished Slalom Path")

        );
    }

}
