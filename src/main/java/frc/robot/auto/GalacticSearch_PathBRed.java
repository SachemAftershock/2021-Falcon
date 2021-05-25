package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ConsoleSubsystem;

public class GalacticSearch_PathBRed extends SequentialCommandGroup {

    public GalacticSearch_PathBRed() {

        // arctan(opposite/adjacent), (opposite = delta letter, adjacent = delta number)
        // hypootneus = adjacent / cos(theta)

        final double AngleC1B3 = -(Math.toDegrees(Math.atan2(-1.0, 2.0))); 
        final double DistanceC1B3 = Math.abs((2 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleC1B3))); 

        final double AngleB3D5 = -(Math.toDegrees(Math.atan2(2.0, 2.0))); 
        final double DistanceB3D5 = Math.abs((2 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleB3D5))); 

        final double AngleD5B7 = -(Math.toDegrees(Math.atan2(-2.0, 2.0))); 
        final double DistanceD5B7 = Math.abs((2 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleD5B7))); 

        final double AngleB7C11 = -(Math.toDegrees(Math.atan2(1.0, 4.0))); 
        final double DistanceB7C11 = Math.abs(((4+1) * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleB7C11))); 

        addCommands(

            // See figure 2.4 Path B pursuing red balls.

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "GalacticSearch: Path B Red selected"),

            // Roll over the 1st red ball, ingest it.  Stop  on its position.
            //new RotateDriveCommand(DriveSubsystem.getInstance(), AngleC1B3 + 5),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceC1B3),

            // Roll over the 2nd red ball, ingest it.  Stop  on its position.
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleB3D5 + 9),//12
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceB3D5 + 5),

            // Roll over the 3rd red ball, ingest it.  Stop  on its position.
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleD5B7 - 5),//-10
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceD5B7),

            // Race through the finish line betwen the goals.
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleB7C11 + 15), // -15
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceB7C11),

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "GalacticSearch: Path B Red completed")
        );
    }

}
