package frc.team0000.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Talon;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.trajectory.timing.TimedState;
import frc.team0000.Constants;
import frc.team0000.loops.Loop;
import frc.team0000.loops.Looper;
import frc.lib.util.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.ErrorCode;


public class Drive extends Subsystem{

    private static Drive instance = new Drive();
    private static final double DRIVE_ENCODER_PPR = 4096.;

    private final TalonSRX leftMaster, rightMaster, leftSlaveA, rightSlaveA;
    private PeriodicIO periodicIO;
    private PigeonIMU pigeon;

    public static Drive getInstance() {
        return instance;
    }

    private Drive() {
        periodicIO = new  PeriodicIO();

        // Start all Talons in open loop mode.
        leftMaster = TalonSRXFactory.createDefaultTalon(Constants.LEFTDriveMasterId);
        configureMaster(leftMaster, false);

        leftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.LEFTDriveMasterId,
                Constants.LEFTDriveSlaveId);
        leftSlaveA.setInverted(true);

        rightMaster = TalonSRXFactory.createDefaultTalon(Constants.RIGHTDriveMasterId);
        configureMaster(rightMaster, false);

        rightSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.RIGHTDriveMasterId,
                Constants.RIGHTDriveSlaveId);
        rightSlaveA.setInverted(true);

        reloadGains();

//        pigeon = new PigeonIMU(leftSlaveB);
//        leftSlaveB.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

        // Force a solenoid message.
//        mIsHighGear = true;
//        setHighGear(false);

        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
//        mIsBrakeMode = true;
//        setBrakeMode(false);

//        mMotionPlanner = new DriveMotionPlanner();
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        VELOCITY_SETPOINT, // velocity PID control
        PATH_FOLLOWING, // used for autonomous driving
        AIM_TO_GOAL, // turn to face the boiler
        TURN_TO_HEADING, // turn in place
        DRIVE_TOWARDS_GOAL_COARSE_ALIGN, // turn to face the boiler, then DRIVE_TOWARDS_GOAL_COARSE_ALIGN
        DRIVE_TOWARDS_GOAL_APPROACH // drive forwards until we are at optimal shooting distance
    }

    private DriveControlState driveControlState;

    private final Loop loop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                setOpenLoop(DriveSignal.NEUTRAL);
//                setBrakeMode(false);
//                setVelocitySetpoint(0, 0);
//                mNavXBoard.reset();
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
                switch (driveControlState) {
                    case OPEN_LOOP:
                        return;
                    case VELOCITY_SETPOINT:
                        return;
//                    case PATH_FOLLOWING:
//                        if (mPathFollower != null) {
//                            updatePathFollower(timestamp);
//                            mCSVWriter.add(mPathFollower.getDebug());
//                        }
//                        return;
//                    case AIM_TO_GOAL:
//                        if (!Superstructure.getInstance().isShooting()) {
//                            updateGoalHeading(timestamp);
//                        }
//                        // fallthrough intended
//                    case TURN_TO_HEADING:
//                        updateTurnToHeading(timestamp);
//                        return;
//                    case DRIVE_TOWARDS_GOAL_COARSE_ALIGN:
//                        updateDriveTowardsGoalCoarseAlign(timestamp);
//                        return;
//                    case DRIVE_TOWARDS_GOAL_APPROACH:
//                        updateDriveTowardsGoalApproach(timestamp);
//                        return;
                    default:
                        System.out.println("Unexpected drive control state: " + driveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
//            mCSVWriter.flush();
        }
    };

    private void configureMaster(TalonSRX talon, boolean left) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
                .CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }
        talon.setInverted(!left);
        talon.setSensorPhase(true);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
//        if (driveControlState != DriveControlState.OPEN_LOOP) {
//            leftMaster.set(ControlMode.PercentOutput,  periodicIO.left_demand, 0.0));
//            rightMaster.set(ControlMode.PercentOutput, periodicIO.left_demand,  0.0));
//            driveControlState = DriveControlState.OPEN_LOOP;
//        }
        // Right side is reversed, but reverseOutput doesn't invert PercentVBus.
        // So set negative on the right master.
//        leftMaster.set(signal.getLeft());
//        rightMaster.set(-signal.getRight());
        leftMaster.set(ControlMode.PercentOutput, 0.5 * signal.getLeft());
        rightMaster.set(ControlMode.PercentOutput, -0.5 * signal.getRight());
    }

    public synchronized void reloadGains() {
        leftMaster.config_kP(0, Constants.LEFT_VEL_KP, Constants.kLongCANTimeoutMs);
        leftMaster.config_kI(0, Constants.LEFT_VEL_KI, Constants.kLongCANTimeoutMs);
        leftMaster.config_kD(0, Constants.LEFT_VEL_KD, Constants.kLongCANTimeoutMs);
        leftMaster.config_kF(0, Constants.LEFT_VEL_KF, Constants.kLongCANTimeoutMs);
        leftMaster.config_IntegralZone(0, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);

        rightMaster.config_kP(0, Constants.RIGHT_VEL_KP, Constants.kLongCANTimeoutMs);
        rightMaster.config_kI(0, Constants.RIGHT_VEL_KI, Constants.kLongCANTimeoutMs);
        rightMaster.config_kD(0, Constants.RIGHT_VEL_KD, Constants.kLongCANTimeoutMs);
        rightMaster.config_kF(0, Constants.RIGHT_VEL_KF, Constants.kLongCANTimeoutMs);
        rightMaster.config_IntegralZone(0, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
    }

    @Override
    public void writeToLog() {
        super.writeToLog();
    }

    @Override
    public void outputToSmartDashboard() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    public enum ShifterState {
        FORCE_LOW_GEAR,
        FORCE_HIGH_GEAR,
        AUTO_SHIFT
    }

    public static class PeriodicIO {
        // INPUTS
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }
}
