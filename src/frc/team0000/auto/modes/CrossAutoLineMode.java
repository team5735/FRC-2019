package frc.team0000.auto.modes;

import frc.team0000.auto.AutoModeBase;
import frc.team0000.auto.AutoModeEndedException;
import frc.team0000.auto.actions.OpenLoopDrive;
import frc.team0000.auto.actions.WaitAction;

public class CrossAutoLineMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cross auto line");
        runAction(new WaitAction(5.0));
        runAction(new OpenLoopDrive(-0.3, -0.3, 5.0, false));
    }
}
