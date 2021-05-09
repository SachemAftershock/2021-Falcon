package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class SetDrivebasePrecisionCommand extends InstantCommand {

    private DriveSubsystem mDrive;
    boolean mPrecision;

    public SetDrivebasePrecisionCommand(DriveSubsystem drive, boolean Precision) {
        mDrive = drive;
        mPrecision = Precision;
        addRequirements(mDrive);
    }

    @Override
    public void execute() {
        if(mPrecision) {
            mDrive.shiftHighGear();   //TODO this looks wrong. Should be precision functions, not gears
        } else {
            mDrive.shiftLowGear();
        }
        System.out.println("SetDrivebasePrecisionCommand: Precision: " + mPrecision);
    }
}
