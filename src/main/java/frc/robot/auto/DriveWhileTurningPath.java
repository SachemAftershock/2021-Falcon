package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWhileTurningPath extends ParallelCommandGroup {

    public DriveWhileTurningPath(double linearInches, double thetaDegrees) {
        addCommands(
            //TODO errorneous, these both need the same subsystem at same time, disallowed. 
            new LinearDriveCommand(DriveSubsystem.getInstance(), linearInches),
            new RotateDriveCommand(DriveSubsystem.getInstance(), thetaDegrees)
        );
    }
}
