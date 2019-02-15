/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {

     /*
          |  --FRONT--  | 
          | |         | | 
       L    |  |   |  |    R
       E  | |  |___|  | |  I
       F  | |    |    | |  G
       T    |   LIFT  |    H
          | |         | |  T
          |   --REAR--  |
    */

    // Wheel stuff for pose
    public static final double kDriveWheelTrackWidthInches = 25.54;
    public static final double kDriveWheelDiameterInches = 3.92820959548 * 0.99; // TODO TUNE
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;

    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0; // kg TODO tune
    public static final double kRobotAngularInertia = 10.0; // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055; // V
    public static final double kDriveKv = 0.135; // V per rad/s
    public static final double kDriveKa = 0.012; // V per rad/s^2

    // Geometry
    public static final double kCenterToFrontBumperDistance = 38.25 / 2.0;
    public static final double kCenterToRearBumperDistance = 38.25 / 2.0;
    public static final double kCenterToSideBumperDistance = 33.75 / 2.0;

    //Path and Pure Pursuit
    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches

    // Motor IDs
    public static final int DRIVETRAIN_CONTROLLER_USB_PORT = 0;
    public static final int SUBSYSTEM_CONTROLLER_USB_PORT = 1;

    public static final int ELEVATOR_MOTOR_ID = 6;
    public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 7;

    public static final int JACK_MOTOR_ID = 14; // tdfaasdfin

    public static final int CARGO_INTAKE_MOTOR_ID = 12;

    public static final int INTAKE_ARM_MOTOR_ID = 10;
    public static final int INTAKE_ARM_MOTOR_FOLLOWER_ID = 11;

    public static final int DRIVETRAIN_RIGHT_MASTER_MOTOR_ID = 2;
    public static final int DRIVETRAIN_RIGHT_FOLLOWER_MOTOR_ID = 3;
    public static final int DRIVETRAIN_LEFT_MASTER_MOTOR_ID = 4;
    public static final int DRIVETRAIN_LEFT_FOLLOWER_MOTOR_ID = 5;

    public static final int INTAKE_ARM_WHEEL_MOTOR_ID = 12;
}
