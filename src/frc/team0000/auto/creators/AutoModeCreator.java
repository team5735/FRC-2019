package frc.team0000.auto.creators;

import frc.team0000.AutoFieldState;
import frc.team0000.auto.AutoModeBase;

public interface AutoModeCreator {
    AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState);
}
