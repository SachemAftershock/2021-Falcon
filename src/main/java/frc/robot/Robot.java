package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private Command mAutonomousCommand;

  private RobotContainer mRobotContainer;

  private AutoSelector mAutoSelector;

  @Override
  public void robotInit() {
      mRobotContainer = RobotContainer.getInstance();
      mRobotContainer.initialize();

      mAutoSelector = new AutoSelector();
      mAutoSelector.selectAuto();
  }

  @Override
  public void robotPeriodic() {
      CommandScheduler.getInstance().run();
      mRobotContainer.periodic();
  }

  @Override
  public void disabledInit(){
  }

  @Override
  public void disabledPeriodic() {
      mAutoSelector.selectAuto();
  }

  @Override
  public void autonomousInit() {
      mRobotContainer.initialize();

      mAutonomousCommand = mAutoSelector.getSelectedAutoCommand();
      if (mAutonomousCommand != null) {
        mAutonomousCommand.schedule();
      }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
      if (mAutonomousCommand != null) {
          mAutonomousCommand.cancel();
      }
      mRobotContainer.initialize(); //TODO: Remove before comp
      System.out.println(Filesystem.getOperatingDirectory());

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() { 
  }
}
