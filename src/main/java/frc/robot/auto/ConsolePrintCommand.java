package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ConsoleSubsystem;

public class ConsolePrintCommand extends InstantCommand {

    private ConsoleSubsystem mConsoleSubsystem;
    private String mOutputMessage;

    public ConsolePrintCommand(ConsoleSubsystem theConsoleSubsystem, String theOutputMessage) {
        mConsoleSubsystem = theConsoleSubsystem;
        mOutputMessage = theOutputMessage;
        addRequirements(mConsoleSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println(mOutputMessage);
    }
    
}
