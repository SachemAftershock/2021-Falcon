package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ConsoleSubsystem;

public class GalacticSearch_PathBBlue extends SequentialCommandGroup {

    public GalacticSearch_PathBBlue() {

        // arctan(opposite/adjacent), (opposite = delta letter, adjacent = delta number)
        // hypootneus = adjacent / cos(theta)

        final double AngleC5D6 = -(Math.toDegrees(Math.atan2(1.0, 1.0)));
        final double DistanceC5D6 = Math.abs((1.0 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleC5D6))); 

        final double AngleD6B8 = -(Math.toDegrees(Math.atan2(-2.0, 2.0))); 
        final double DistanceD6B8 = Math.abs((2.0 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleD6B8))); 

        final double AngleB8D10 = -(Math.toDegrees(Math.atan2(2.0, 2.0))); 
        final double DistanceB8D10 = Math.abs((2.0 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleB8D10))); 

        final double AngleD10C11 = -(Math.toDegrees(Math.atan2(-1.0, 1.0))); 
        final double DistanceD10C11 = Math.abs((1.0 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleD10C11))); 

        addCommands(

            // See figure 2.4 Path B pursuing red balls.

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "GalacticSearch: Path B Blue selected"),

            // Aim at D6, then go get blue ball 1 at it.  Stop at that position.
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleC5D6),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceC5D6),

            // Aim at B8, then go get blue ball 2 at it.  Stop at that position.
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleD6B8),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceD6B8), 

            // Aim at D10, then go get blue ball 3 at it.  Stop at that position.
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleB8D10),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceB8D10), 

            // Aim at center of finish line (C11) and drive through it!
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleD10C11),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceD10C11),

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "GalacticSearch: Path B Blue completed")
        );
    }

}
