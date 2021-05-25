package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ConsoleSubsystem;

public class AutoNavChallengeBarrelRacing extends SequentialCommandGroup {

    public AutoNavChallengeBarrelRacing () {

        addCommands(
    
            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "Started Barrel Racing"),

            // Robot starts at C1, facing eastward, relative to Figure 2-5.

            // Get into position to start the first cone (D5)
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kEastward ),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 5),  // C1 to C6
           
            // Loop down and clockwise around D5 
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kSouthward),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2),  // C6 to E6
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kWesthward),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2),  // E6 to E4
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kNorthward),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2 - 4),  // E4 to C4            

            //  Get into position to start the second cone (B8)
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kEastward + 3), 
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2),  // C4 to C9
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kEastward + 3), 
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2),  // C4 to C9
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kEastward + 3), 
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 1 + 5),  // C4 to C9

            // Loop up and CCW around B8, then shoot down to start position for third code (D10)
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kNorthward),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2 - 18 ), //-25 // C9 to A9
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kWesthward),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2),  // A9 to A7
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kSouthward),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 4 ),  // +25 // A7 to E7

            // Loop around D10
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kEastward - 5),
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 4 - 10),  // E7 to E11
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kNorthward), 
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 2 - 10),  // E11 to C11

            // Race into to endzone!
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kWesthward + 4 ), //+10 //This is 170 degrees
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 3), // E11 to C1
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kWesthward + 4 ), //+10 //This is 170 degrees
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 3), // E11 to C1
            new RotateDriveCommand(DriveSubsystem.getInstance(), Map.kWesthward + 4 ), //+10 //This is 170 degrees
            new LinearDriveCommand(DriveSubsystem.getInstance(), Map.kNavPointInches * 4 - 15), // E11 to C1

            new ConsolePrintCommand(ConsoleSubsystem.getInstance(), "Finished Barrel Racing")            );
    }

}
