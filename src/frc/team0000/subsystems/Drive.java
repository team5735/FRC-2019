package frc.team0000.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.TimedState;
import frc.team0000.Constants;
import frc.team0000.RobotState;
import frc.team0000.loops.ILooper;
import frc.team0000.loops.Loop;
import frc.team0000.loops.Looper;
import frc.lib.util.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.ErrorCode;
import frc.team0000.planners.DriveMotionPlanner;


public class Drive extends Subsystem{
    private static Drive instance = new Drive();
    private static final double DRIVE_ENCODER_PPR = 4096.;

    private final TalonSRX leftMaster, rightMaster, leftSlaveA, rightSlaveA;
    private PeriodicIO periodicIO;
    private Rotation2d gyroOffset = Rotation2d.identity();
    private DriveMotionPlanner motionPlanner;
//    private PigeonIMU pigeon;
    private boolean overrideTrajectory = false;

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

        motionPlanner = new DriveMotionPlanner();
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
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
                        break;
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
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
        if (driveControlState != DriveControlState.OPEN_LOOP) {

            System.out.println("Switching to open loop");
            System.out.println(signal);
            driveControlState = DriveControlState.OPEN_LOOP;
            leftMaster.configNeutralDeadband(0.04, 0);
            rightMaster.configNeutralDeadband(0.04, 0);
        }
        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
        periodicIO.left_feedforward = 0.0;
        periodicIO.right_feedforward = 0.0;
    }

    public synchronized void reloadGains() {
        leftMaster.config_kP(0, Constants.LEFT_VEL_KP, Constants.kLongCANTimeoutMs);
        leftMaster.config_kI(0, Constants.LEFT_VEL_KI, Constants.kLongCANTimeoutMs);
        leftMaster.config_kD(0, Constants.LEFT_VEL_KD, Constants.kLongCANTimeoutMs);
        leftMaster.config_kF(0, Constants.LEFT_VEL_KF, Constants.kLongCANTimeoutMs);
//        leftMaster.config_IntegralZone(0, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
        rightMaster.config_IntegralZone(0, 0, Constants.kLongCANTimeoutMs);

        rightMaster.config_kP(0, Constants.RIGHT_VEL_KP, Constants.kLongCANTimeoutMs);
        rightMaster.config_kI(0, Constants.RIGHT_VEL_KI, Constants.kLongCANTimeoutMs);
        rightMaster.config_kD(0, Constants.RIGHT_VEL_KD, Constants.kLongCANTimeoutMs);
        rightMaster.config_kF(0, Constants.RIGHT_VEL_KF, Constants.kLongCANTimeoutMs);
//        rightMaster.config_IntegralZone(0, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
        rightMaster.config_IntegralZone(0, 0, Constants.kLongCANTimeoutMs);
    }

    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (driveControlState != DriveControlState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            leftMaster.selectProfileSlot(0, 0);
            rightMaster.selectProfileSlot(0, 0);
            leftMaster.configNeutralDeadband(0.0, 0);
            rightMaster.configNeutralDeadband(0.0, 0);

            driveControlState = DriveControlState.PATH_FOLLOWING;
        }
        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
        periodicIO.left_feedforward = feedforward.getLeft();
        periodicIO.right_feedforward = feedforward.getRight();
    }

    @Override
    public void writeToLog() {
        super.writeToLog();
    }

    @Override
    public void registerEnabledLoops(ILooper loopIn) {
        loopIn.register(loop);
    }

    private void updatePathFollower() {
        if (driveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = motionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

//             DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            periodicIO.error = motionPlanner.error();
            periodicIO.path_setpoint = motionPlanner.setpoint();

            if (!overrideTrajectory) {
                setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                periodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                periodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                periodicIO.left_accel = periodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public synchronized Rotation2d getHeading() {
        return periodicIO.gyro_heading;
    }
    
    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

//        gyroOffset = heading.rotateBy(Rotation2d.fromDegrees(pigeon.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + gyroOffset.getDegrees());

        periodicIO.gyro_heading = heading;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", periodicIO.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", periodicIO.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", periodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", periodicIO.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());
//
        SmartDashboard.putNumber("x err", periodicIO.error.getTranslation().x());
        SmartDashboard.putNumber("y err", periodicIO.error.getTranslation().y());
        SmartDashboard.putNumber("theta err", periodicIO.error.getRotation().getDegrees());
//        if (getHeading() != null) {
//            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
//        }
//        if (mCSVWriter != null) {
//            mCSVWriter.write();
//        }
    }

    public synchronized void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);
        periodicIO = new PeriodicIO();
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    public double getLeftEncoderRotations() {
        return periodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return periodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityNativeUnits() {
        return periodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLeftVelocityNativeUnits() {
        return periodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public void overrideTrajectory(boolean value) {
        overrideTrajectory = value;
    }

    @Override
    public boolean checkSystem() {
//        TODO FIX
//        boolean leftSide = TalonSRXChecker.CheckTalons(this,
//                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
//                    {
//                        add(new TalonSRXChecker.TalonSRXConfig("left_master", mLeftMaster));
//                        add(new TalonSRXChecker.TalonSRXConfig("left_slave", mLeftSlaveA));
//                        add(new TalonSRXChecker.TalonSRXConfig("left_slave1", mLeftSlaveB));
//                    }
//                }, new TalonSRXChecker.CheckerConfig() {
//                    {
//                        mCurrentFloor = 2;
//                        mRPMFloor = 1500;
//                        mCurrentEpsilon = 2.0;
//                        mRPMEpsilon = 250;
//                        mRPMSupplier = () -> mLeftMaster.getSelectedSensorVelocity(0);
//                    }
//                });
//        boolean rightSide = TalonSRXChecker.CheckTalons(this,
//                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
//                    {
//                        add(new TalonSRXChecker.TalonSRXConfig("right_master", mRightMaster));
//                        add(new TalonSRXChecker.TalonSRXConfig("right_slave", mRightSlaveA));
//                        add(new TalonSRXChecker.TalonSRXConfig("right_slave1", mRightSlaveB));
//                    }
//                }, new TalonSRXChecker.CheckerConfig() {
//                    {
//                        mCurrentFloor = 2;
//                        mRPMFloor = 1500;
//                        mCurrentEpsilon = 2.0;
//                        mRPMEpsilon = 250;
//                        mRPMSupplier = () -> mRightMaster.getSelectedSensorVelocity(0);
//                    }
//                });
//        return leftSide && rightSide;
        return true;
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (motionPlanner != null) {
            overrideTrajectory = false;
            motionPlanner.reset();
            motionPlanner.setTrajectory(trajectory);
            driveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (motionPlanner == null || driveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return motionPlanner.isDone() || overrideTrajectory;
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
