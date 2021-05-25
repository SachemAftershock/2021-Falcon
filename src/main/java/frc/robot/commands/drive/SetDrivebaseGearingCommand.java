package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class SetDrivebaseGearingCommand extends InstantCommand {

    private DriveSubsystem mDrive;
    boolean mHighGear;

    public SetDrivebaseGearingCommand(DriveSubsystem drive, boolean HighGear) {
        mDrive = drive;
        mHighGear = HighGear;
        addRequirements(mDrive);
    }

    @Override
    public void execute() {
        if(mHighGear) {
            mDrive.shiftHighGear();
        } else {
            mDrive.shiftLowGear();
        }
        System.out.println("SetDrivebaseGearingCommand: Precision: " + mHighGear);
    }
}
