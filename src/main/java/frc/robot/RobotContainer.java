// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.AftershockXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.drive.ManualDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.commands.drive.ToggleCollisionAvoidanceCommand;
import frc.robot.commands.drive.ToggleDrivebaseGearingCommand;
import frc.robot.commands.drive.ToggleManualDriveInversionCommand;
import frc.robot.commands.drive.TogglePrecisionDrivingCommand;
import frc.robot.subsystems.CollisionAvoidanceSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PowerSubsystem;
import frc.lib.SubsystemManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer mInstance;

  private final AftershockXboxController mControllerPrimary = new AftershockXboxController(ControllerConstants.kControllerPrimaryId);
  private final AftershockXboxController mControllerSecondary = new AftershockXboxController(ControllerConstants.kControllerSecondaryId);

  private final CollisionAvoidanceSubsystem mCollisionAvoidance = CollisionAvoidanceSubsystem.getInstance();
  private final DriveSubsystem mDrive = DriveSubsystem.getInstance();
  private final PowerSubsystem mPower = PowerSubsystem.getInstance();

  private final SubsystemManager mSubsystemManager;

  // Primary Controller
  private JoystickButton bToggleDriveGear;
  private JoystickButton bToggleDriveInversion;
  private JoystickButton bToggleCollisionAvoidance;
  private JoystickButton bTogglePrecisionDrive;
  private JoystickButton bClearCommandQueuePrimary;

  // Secondary Controller
  private boolean isLowPowerMode;
  private boolean lastPowerMode;
  private JoystickButton bClearCommandQueueSecondary;
  private JoystickButton bDeployIntake;
  private JoystickButton bRetractIntake;
  private JoystickButton bEjectIntake;
  private JoystickButton bIngestIntake;
  private JoystickButton bArm;

  private int iterationCounter;
  private final int kMaxIterations = 5000 / 20;

  /**
   * Constructor for RobotCotainer Class
   */
  private RobotContainer() {
      mSubsystemManager = SubsystemManager.getInstance();
      mSubsystemManager.setSubsystems(
          mCollisionAvoidance,
          mDrive,
          mPower
      );

      isLowPowerMode = false; // Robot starts in On mode (not low power)
      lastPowerMode = !isLowPowerMode; // Set so first trigger detection will cause toggle of mode.

      configureButtonBindings();
      CommandScheduler.getInstance().setDefaultCommand(mDrive, new ManualDriveCommand(mDrive, mControllerPrimary));

      iterationCounter = kMaxIterations;
  }

  /**
   * RobotContainer Initialization, Runs at Robot Powered On
   */
  public void initialize() {
    mSubsystemManager.initialize();
}

/**
   * Maps Buttons on Primary & Secondary Controllers to Commands
   */
  private void configureButtonBindings() {

      // PRIMARY CONTROLLER

      bToggleDriveGear = new JoystickButton(mControllerPrimary, XboxController.Button.kA.value);
      bToggleDriveGear.whenPressed(new ToggleDrivebaseGearingCommand(mDrive));

      bTogglePrecisionDrive = new JoystickButton(mControllerPrimary, XboxController.Button.kX.value);
      bTogglePrecisionDrive.whenPressed(new TogglePrecisionDrivingCommand(mDrive));

      bToggleDriveInversion = new JoystickButton(mControllerPrimary, XboxController.Button.kB.value);
      bToggleDriveInversion.whenPressed(new ToggleManualDriveInversionCommand(mDrive));

      bToggleCollisionAvoidance = new JoystickButton(mControllerPrimary, XboxController.Button.kBack.value);
      bToggleCollisionAvoidance.whenPressed(new ToggleCollisionAvoidanceCommand(mCollisionAvoidance, mControllerPrimary));

      bClearCommandQueuePrimary = new JoystickButton(mControllerPrimary, XboxController.Button.kStart.value);
      bClearCommandQueuePrimary.whenPressed(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

      // SECONDARY CONTROLLER

      bClearCommandQueueSecondary = new JoystickButton(mControllerSecondary, XboxController.Button.kStart.value);
      bClearCommandQueueSecondary.whenPressed(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    }


  /**
   * Checks Button Status Periodically to execute Commands
   * <p>
   * Used for non-traditional buttons (D-Pad,Triggers)
   */
  public void periodic() {

    // While not auto-rotating, D-Pad can be used to command to 8 direction robot angle.
    if(mControllerPrimary.getDPadPressed() && !mDrive.isAutoRotateRunning()) {
        CommandScheduler.getInstance().schedule(new RotateDriveCommand(mDrive, mControllerPrimary.getDPadAngle()));
    }

    if (--iterationCounter <= 0) {
        mSubsystemManager.outputTelemetry();
        iterationCounter = kMaxIterations;
    }
  }
    
  /**
    * Gets Xbox Controller for the Primary Driver, tasked with driving the robot
    * 
    * @return Primary Xbox Controller
    */
  public AftershockXboxController getControllerPrimary() {
        return mControllerPrimary;
  }

  /**
   * Gets Xbox Controller for the Secondary Driver, tasked with controlling all mechanisms
   * 
   * @return Secondary Xbox Controller
   */
  public AftershockXboxController getControllerSecondary() {
      return mControllerSecondary;
  }

  /**
   * @return RobotContainer Singleton Instance
   */
  public synchronized static RobotContainer getInstance() {
      if(mInstance == null) {
          mInstance = new RobotContainer();
      }
      return mInstance;
  }
}

