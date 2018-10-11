package frc.team0000;

public class Constants {
    public static double kLooperDt = 0.005;

    public static final int LEFTDriveMasterId = 1;
    public static final int LEFTDriveSlaveId = 2;
    public static final int RIGHTDriveMasterId = 3;
    public static final int RIGHTDriveSlaveId = 4;

    public static double kDriveLowGearVelocityKp = 1.0;
    public static double kDriveLowGearVelocityKi = 0.002;
    public static double kDriveLowGearVelocityKd = 100.0;
    public static double kDriveLowGearVelocityKf = .45;
    public static int kDriveLowGearVelocityIZone = 0;

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

}
