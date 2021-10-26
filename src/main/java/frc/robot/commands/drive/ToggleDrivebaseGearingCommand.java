package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ToggleDrivebaseGearingCommand extends InstantCommand {

    private DriveSubsystem mDrive;

    public ToggleDrivebaseGearingCommand(DriveSubsystem drive) {
        mDrive = drive;
        addRequirements(mDrive);
    }

    /*
    @Override
    public void execute() {
        if(mDrive.isHighGear()) {
            mDrive.shiftLowGear();
        } else {
            mDrive.shiftHighGear();
        }
        System.out.println("ToggleDrivebaseGearingCommand: High Gear: " + mDrive.isHighGear());
    }
    */
}
