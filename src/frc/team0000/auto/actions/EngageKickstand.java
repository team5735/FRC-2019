package frc.team0000.auto.actions;

import frc.team0000.subsystems.Superstructure;

public class EngageKickstand extends RunOnceAction {
    private boolean mEngage;

    public EngageKickstand(boolean engage) {
        mEngage = engage;
    }

    @Override
    public void runOnce() {
        Superstructure.getInstance().setKickstand(mEngage);
    }
}
