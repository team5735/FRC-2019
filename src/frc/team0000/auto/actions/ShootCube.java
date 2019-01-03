package frc.team0000.auto.actions;

import frc.team0000.statemachines.IntakeStateMachine;
import frc.team0000.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class ShootCube implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private static final double kShootTime = 0.35;
    private final double mPower;

    private double mStartTime;

    public ShootCube(double power) {
        mPower = power;
    }

    public ShootCube() {
        mPower = 1.0;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mIntake.shoot(mPower);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > kShootTime;
    }

    @Override
    public void done() {
        mIntake.setState(IntakeStateMachine.WantedAction.WANT_MANUAL);
        mIntake.setPower(0.0);
    }
}
