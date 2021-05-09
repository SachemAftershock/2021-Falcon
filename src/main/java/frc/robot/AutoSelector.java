package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.ComplexAutoPath;
import frc.robot.auto.RamseteTestAutoPath;
import frc.robot.auto.GalacticSearch_Begin;
import frc.robot.auto.StraightThenRotateAutoPath;
import frc.robot.auto.AutoNavChallengeBarrelRacing;
import frc.robot.auto.AutoNavChallengeSlalomPath;
import frc.robot.auto.AutoNavChallengeBouncePath;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.subsystems.DriveSubsystem;


/**
 * Class to select desired Autonomous Configuration at Startup
 * 
 * @author Shreyas Prasad
 */
public class AutoSelector {

    /**
     * Autonomous Paths that map to a Command Sequence
     * 
     * @author Shreyas Prasad
     */
    enum AutoPath {
        eNothing, eStraight, eStraightThenTurn, eComplexPath, eRamseteTest, 
        eGalacticSearch, eAutoNav_BarrelRacing, 
        eAutoNav_SlalomPath, eAutoNav_BouncePath, eHyperdrive 
    }

    private AutoPath mSelectedAutoScenario, mPrevAutoScenario;

    private SendableChooser<AutoPath> mAutoChooser;

    /**
     * Constructor for AutoSelector Class
     */
    public AutoSelector() {
        mPrevAutoScenario = null;

        mAutoChooser = new SendableChooser<>();
        mAutoChooser.setDefaultOption("No Path", AutoPath.eNothing);
        mAutoChooser.addOption("Straight", AutoPath.eStraight);
        mAutoChooser.addOption("Straight then Turn", AutoPath.eStraightThenTurn);
        mAutoChooser.addOption("Complex Path", AutoPath.eComplexPath);
        mAutoChooser.addOption("RamseteTest Path", AutoPath.eRamseteTest);
        mAutoChooser.addOption("Galactic Search", AutoPath.eGalacticSearch);
        mAutoChooser.addOption("Barrel Racing", AutoPath.eAutoNav_BarrelRacing);
        mAutoChooser.addOption("Slalom Path", AutoPath.eAutoNav_SlalomPath);
        mAutoChooser.addOption("Bounce Path", AutoPath.eAutoNav_BouncePath);
        mAutoChooser.addOption("Hyperdrive", AutoPath.eHyperdrive);

        SmartDashboard.putData("Auto Path", mAutoChooser);
    }

    /**
     * Allows Operators to select one of several designed Autonomous Routines via SmartDashboard/Shuffleboard
     */
    public void selectAuto() {
        mSelectedAutoScenario = mAutoChooser.getSelected();
        if(mPrevAutoScenario != mSelectedAutoScenario) {
            System.out.println("Changing Auto Path: " + mSelectedAutoScenario.name());
        }
        mPrevAutoScenario = mSelectedAutoScenario;
    }

    /**
     * Decodes AutoPath Enum to a Command Sequence
     * 
     * @return Command Sequence for Autonomous
     */
    public Command getSelectedAutoCommand() {
        switch(mSelectedAutoScenario) {
            case eStraight:
                return new RotateDriveCommand(DriveSubsystem.getInstance(), 90);
                //return new LinearDriveCommand(DriveSubsystem.getInstance(), 30);
            case eStraightThenTurn:
                return new StraightThenRotateAutoPath();
            case eComplexPath:
                return new ComplexAutoPath();
            case eRamseteTest:
                return (new RamseteTestAutoPath(DriveSubsystem.getInstance())).getCommand();
            case eGalacticSearch:
                return new GalacticSearch_Begin();
            case eAutoNav_BarrelRacing:
                return (new AutoNavChallengeBarrelRacing());
            case eAutoNav_SlalomPath:
                return (new AutoNavChallengeSlalomPath());
            case eAutoNav_BouncePath:
                return (new AutoNavChallengeBouncePath());
//TODO            case eHyperdrive:
//                return (new RamseteTestAutoPath(DriveSubsystem.getInstance())).getCommand();
            case eNothing:
            default:
                return null;
        }
    }
}