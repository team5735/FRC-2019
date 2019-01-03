package frc.team0000;

import edu.wpi.first.wpilibj.Solenoid;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class Constants {

    public static final double kLooperDt = 0.01;

    public static final int LEFTDriveMasterId = 1;
    public static final int LEFTDriveSlaveId = 2;
    public static final int RIGHTDriveMasterId = 3;
    public static final int RIGHTDriveSlaveId = 4;
    // PID LEFT CLOSED LOOP VELOCITY
    public static final double
            LEFT_VEL_KF = 0.25 * 1023 / 825,
            LEFT_VEL_KP = 1.2,
            LEFT_VEL_KI = 0,
            LEFT_VEL_KD = 0;
    // PID LEFT CLOSED LOOP VELOCITY
    public static final double
            RIGHT_VEL_KF = 0.25 * 1023 / 685,
            RIGHT_VEL_KP = 1.92,
            RIGHT_VEL_KI = 0,
            RIGHT_VEL_KD = 0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100;
    // Wheels
    public static final double kDriveWheelTrackWidthInches = 25.54;
    public static final double kDriveWheelDiameterInches = 3.92820959548 * 0.99;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!
    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune

    /* ROBOT PHYSICAL CONSTANTS */
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.135;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2
    // Geometry
    public static final double kCenterToFrontBumperDistance = 38.25 / 2.0;
    public static final double kCenterToRearBumperDistance = 38.25 / 2.0;
    public static final double kCenterToSideBumperDistance = 33.75 / 2.0;
    // Pose of the LIDAR frame w.r.t. the robot frame
    // TODO measure in CAD/on robot!
    public static final double kLidarXOffset = -3.3211;
    public static final double kLidarYOffset = 0.0;
    public static final double kLidarYawAngleDegrees = 0.0;
    /* LIDAR CONSTANTS */
    public static final int kChezyLidarScanSize = 400;
    public static final int kChezyLidarNumScansToStore = 10;
    public static final String kChezyLidarPath = "/home/root/chezy_lidar";
    public static final double kChezyLidarRestartTime = 2.5;
    public static final String kLidarLogDir = "/home/lvuser/lidarLogs/";
    public static final int kNumLidarLogsToKeep = 10;
    public static final double kLidarICPTranslationEpsilon = 0.01; // convergence threshold for tx,ty
    public static final double kLidarICPAngleEpsilon = 0.01;       // convergence threshold for theta
    public static final int kCameraStreamPort = 5810;
    /* LIDAR CONSTANTS */
    public static final double kScaleTrackerTimeout = 0.6;
    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second
    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches

    /* CONTROL LOOP GAINS */
    // Drive
    public static final int kLeftDriveMasterId = 5;
    public static final int kLeftDriveSlaveAId = 6;
    public static final int kLeftDriveSlaveBId = 7;
    public static final int kRightDriveMasterId = 12;

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    public static final int kRightDriveSlaveAId = 13;
    public static final int kRightDriveSlaveBId = 14;
    // Followers
    public static final int kFollowerLeftAChannelId = 2;
    public static final int kFollowerLeftBChannelId = 3;
    public static final int kFollowerRightAChannelId = 0;
    public static final int kFollowerRightBChannelId = 1;
    public static final int kFollowerRearAChannelId = 4;
    public static final int kFollowerRearBChannelId = 5;
    // Intake
    public static final int kIntakeLeftMasterId = 9;
    public static final int kIntakeRightMasterId = 10;
    public static final int kCanifierId = 0;
    // Elevator
    public static final int kElevatorMasterId = 11;
    public static final int kElevatorRightSlaveId = 8;
    public static final int kElevatorLeftSlaveAId = 1;
    public static final int kElevatorLeftSlaveBId = 2;
    // Wrist
    public static final int KWristMasterId = 15;
    // Solenoids
    public static final int kShifterSolenoidId = 12; // PCM 0, Solenoid 4
    public static final int kIntakeCloseSolenoid = 10;
    public static final int kIntakeClampSolenoid = 9;
    public static final int kForkliftDeploySolenoid = 7;  // CURRENTLY 6 ON PRACTICE!!!
    public static final int kFollowerWheelSolenoid = 11;
    public static final int kElevatorShifterSolenoidId = 8;
    public static final int kUnlockHookSolenoid = 4;
    public static final int kJazzHandsSolenoid = 5;
    public static final int kKickstandSolenoid = 3;
    // Control Board
    public static final boolean kUseGamepadForDriving = false;
    public static final boolean kUseGamepadForButtons = true;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.5;
    public static final double kJoystickJogThreshold = 0.4;
    // Height in in after applying turn factor.
    public static final double kElevatorLowSensitivityThreshold = 50.0;
    public static final double kLowSensitivityFactor = 1.0 / 4.0;
    public static final double kElevatorThrottleDeadband = 0.3;
    public static final double kMinShootTimeSec = 0.2;
    public static final double kJazzHandsEpsilon = 2.5;
    public static final double kKickstandToggleRumbleTime = 0.5;
    public static final double kKickstandDelay = 1.0;
    public static double kLooperDt = 0.005;
    public static double kDriveLowGearVelocityKp = 1.0;
    public static double kDriveLowGearVelocityKi = 0.002;
    public static double kDriveLowGearVelocityKd = 100.0;
    public static double kDriveLowGearVelocityKf = .45;
    public static int kDriveLowGearVelocityIZone = 0;

    /**
     * Make an {@link Solenoid} instance for the single-number ID of the solenoid
     * <p>
     * Solenoids were wired in an inane method and also not labeled zero indexed.
     * <p>
     * Solenoids 1-4 are on PCM 1, Solenoids 7-4.
     * Solenoids 5-8 are on PCM 0, Solenoids 0-3.
     * Solenoids 9-12 are on PCM 0, Solenoids 7-4.
     *
     * @param solenoidId One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId) {
        if (solenoidId <= 4) {
            // These solenoids are on PCM 1, wired 1-4 to 7-4.
            return new Solenoid(1, 8 - solenoidId);
        } else if (solenoidId <= 8) {
            // These solenoids are on PCM 0, wired 5-8 to 0-3.
            return new Solenoid(0, solenoidId - 5);
        } else if (solenoidId <= 12) {
            // These solenoids are on PCM 0, wired 9-12 to 7-4.
            return new Solenoid(0, 16 - solenoidId);
        }
        throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
    }

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}
