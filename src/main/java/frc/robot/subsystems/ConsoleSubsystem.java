package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;

public class ConsoleSubsystem extends AftershockSubsystem {

    private static ConsoleSubsystem mInstance;

    private ConsoleSubsystem() {
        super();
        setName("Console Subsystem");

    }
    
    @Override
    public void initialize() {}


    @Override
    public void outputTelemetry() { }

    @Override
    public boolean checkSystem() {
        return true;
    }

    /**
     * @return  Singleton Instance
     */
    public synchronized static ConsoleSubsystem getInstance() {
        if(mInstance == null) {
            mInstance = new ConsoleSubsystem();
        }
        return mInstance;
    }
}
