package frc.team0000.subsystems;

import frc.team0000.loops.Looper;

public class SuperStructure extends Subsystem{

    private static SuperStructure instance = new SuperStructure();

    public static SuperStructure getInstance() {
        return instance;
    }

    private final Drive mDrive = Drive.getInstance();

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {

    }
}
