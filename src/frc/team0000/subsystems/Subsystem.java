package frc.team0000.subsystems;

import frc.team0000.loops.Looper;

public abstract class Subsystem {
    public void writeToLog() {
    };

    public abstract void outputToSmartDashboard();

    public abstract void stop();

    public abstract void zeroSensors();

    public abstract void registerEnabledLoops(Looper enabledLooper);
}
