/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team0000;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CheesyDriveHelper;
import frc.lib.util.CrashTracker;
import frc.lib.util.DriveSignal;
import frc.team0000.subsystems.*;
import frc.team0000.loops.*;

import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
// If you rename or move this class, update the build.properties file in the project root
public class Robot extends TimedRobot {

    private Drive drive = Drive.getInstance();
    private SuperStructure superstructure = SuperStructure.getInstance();
    private RobotState robotState = RobotState.getInstance();

    private final SubsystemManager subsystemManager = new SubsystemManager(
            Arrays.asList(Drive.getInstance(), superstructure.getInstance()));

    private Looper enabledLooper = new Looper();
    private CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();



    private ControlBoardInterface controlBoard = ControlBoard.getInstance();
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            subsystemManager.registerEnabledLoops(enabledLooper);
//            enabledLooper.register(VisionProcessor.getInstance());
//            enabledLooper.register(RobotStateEstimator.getInstance());

//            mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());

//            AutoModeSelector.initAutoModeSelector();

//            mDelayedAimButton = new DelayedBoolean(Timer.getFPGATimestamp(), 0.1);
//             Force an true update now to prevent robot from running at start.
//            mDelayedAimButton.update(Timer.getFPGATimestamp(), true);

            // Pre calculate the paths we use for auto.
//            PathAdapter.calculatePaths();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() {
        
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

            // Start loopers
            enabledLooper.start();
            drive.setOpenLoop(DriveSignal.NEUTRAL);
//            mDrive.setBrakeMode(false);
            // Shift to high
//            mDrive.setHighGear(true);
//            zeroAllSensors();
//            mSuperstructure.reloadConstants();
//            mSuperstructure.setOverrideCompressor(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        double throttle = controlBoard.getThrottle();
        double turn = controlBoard.getTurn();
        drive.setOpenLoop(cheesyDriveHelper.cheesyDrive(throttle, turn, controlBoard.getQuickTurn(),
                false));
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        
    }

        public void allPeriodic() {
//            robotState.outputToSmartDashboard();
            subsystemManager.outputToSmartDashboard();
            subsystemManager.writeToLog();
//            .outputToSmartDashboard();
//            SmartDashboard.putBoolean("camera_connected", mVisionServer.isConnected());

//            ConnectionMonitor.getInstance().setLastPacketTime(Timer.getFPGATimestamp());
        }

}
