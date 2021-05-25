package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.commands.drive.SetDrivebaseGearingCommand;
import frc.robot.commands.drive.SetDrivebasePrecisionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ConsoleSubsystem;

public class GalacticSearch_PathARed extends SequentialCommandGroup {

    public GalacticSearch_PathARed() {

        // arctan(opposite/adjacent), (opposite = delta letter, adjacent = delta number)
        // hypootneus = adjacent / cos(theta)

        final double AngleC3D5 = -(Math.toDegrees(Math.atan2(1.0, 2.0))); 
        final double DistanceC3D5 = Math.abs((2.0 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleC3D5))); 

        final double AngleD5A6 = -(Math.toDegrees(Math.atan2(-3.0, 1.0))); 
        final double DistanceD5A6 = Math.abs((1.0 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleD5A6))); 

        final double AngleA6C11 = -(Math.toDegrees(Math.atan2(2.0, 5.0))); 
        final double DistanceA6C11 = Math.abs((5.0 * Map.kNavPointInches) / Math.cos(Math.toRadians(AngleA6C11))); // add some to cross deeper.

        addCommands(

            // See figure 2.3 Path A pursuing red balls.

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "GalacticSearch: Path A Red selected"),

//            new SetDrivebaseGearingCommand(DriveSubsystem.getInstance(), true),
//            new SetDrivebasePrecisionCommand(DriveSubsystem.getInstance(), true),

            // Roll over the 1st red ball, ingest it. Stop on its position. 
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2), // C1 to C3

            // Roll over the 2nd red ball, ingest it. Stop on its position. 
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleC3D5 + 10),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceC3D5),

            // Roll over the 3rd red ball, ingest it. Stop on its position. 
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleD5A6 - 3),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceD5A6),

            // Drive through the finish line between the goals.
            new RotateDriveCommand(DriveSubsystem.getInstance(), AngleA6C11),
            //new SetDrivebaseGearingCommand(DriveSubsystem.getInstance(), true),
            //new SetDrivebasePrecisionCommand(DriveSubsystem.getInstance(), true),
            new LinearDriveCommand(DriveSubsystem.getInstance(), DistanceA6C11),

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "GalacticSearch: Path A Red completed")
        );
    }

}
