package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ConsoleSubsystem;

public class GalacticSearch_PathABlue extends SequentialCommandGroup {

    public GalacticSearch_PathABlue() {

        // arctan(opposite/adjacent), (opposite = delta letter, adjacent = delta number)
        // hypootneus = adjacent / cos(theta)

        final double AngleC5E6 = -(Math.toDegrees(Math.atan2(2.0, 1.0))); // arctan(opposite/adjacent)
        final double DistanceC5E6 = Math.abs((1.0 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleC5E6))); // hypootneus = adjacent / cos(theta) 

        final double AngleE6B7 = -(Math.toDegrees(Math.atan2(-3.0, 1.0))); // arctan(opposite/adjacent)
        final double DistanceE6B7 = Math.abs((1.0 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleE6B7))); // hypootneus = adjacent / cos(theta) 

        final double AngleB7C9 = -(Math.toDegrees(Math.atan2(1.0, 2.0))); // arctan(opposite/adjacent)
        final double DistanceB7C9 = Math.abs((2.0 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleB7C9))); // hypootneus = adjacent / cos(theta) 

        final double AngleC9C11 = 0.0;
        final double DistanceC9C11 = 2;

        addCommands(
            
            // See figure 2.3 Path A pursuing blue balls.

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "GalacticSearch: Path A Blue selected"),

            // Go to and drive over 1st blue ball, ingest it. Stop on its position.
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleC5E6),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceC5E6),

            // Turn towards then roll over 3rd blue ball, ingest it, stop on its position.
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleE6B7),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceE6B7), 


            // Turn towards then roll over 3rd blue ball, ingest it, stop on its position.
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleB7C9),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceB7C9), 

            // Drive through the finish line, between the goals!
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleC9C11),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceC9C11),  

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "GalacticSearch: Path A Blue completed")
        );
    }

}
