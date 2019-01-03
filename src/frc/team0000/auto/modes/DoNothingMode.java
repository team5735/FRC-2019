package frc.team0000.auto.modes;

import frc.team0000.auto.AutoModeBase;
import frc.team0000.auto.AutoModeEndedException;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Doing nothing");
    }
}
