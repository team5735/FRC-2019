/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryUtil;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class TrajectoryGenerator extends Subsystem {
  private static final double kMaxVelocity = 130.0;
  private static final double kMaxAccel = 130.0;
  private static final double kMaxCentripetalAccelElevatorDown = 110.0;
  private static final double kMaxCentripetalAccel = 100.0;
  private static final double kMaxVoltage = 9.0;

  private TrajectorySet mTrajectorySet = null;

  public TrajectoryGenerator() {
    // mMotionPlanner = new DriveMotionPlanner();
  }

  public void generateTrajectories() {
    if (mTrajectorySet == null) {
      System.out.println("Generating trajectories...");
      mTrajectorySet = new TrajectorySet();
      System.out.println("Finished trajectory generation");
    }
  }

  public TrajectorySet getTrajectorySet() {
    return mTrajectorySet;
  }

  public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed, final List<Pose2d> waypoints,
      final List<TimingConstraint<Pose2dWithCurvature>> constraints, double max_vel, // inches/s
      double max_accel, // inches/s^2
      double max_voltage) {
    return Robot.drive.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
  }

  public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed, final List<Pose2d> waypoints,
      final List<TimingConstraint<Pose2dWithCurvature>> constraints, double start_vel, // inches/s
      double end_vel, // inches/s
      double max_vel, // inches/s
      double max_accel, // inches/s^2
      double max_voltage) {
    return Robot.drive.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel,
        max_voltage);
  }

  // CRITICAL POSES
  // Origin is the center of the robot when the robot is placed against the middle
  // of the alliance station wall.
  // +x is towards the center of the field.
  // +y is to the left.
  // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x
  // axis for LEFT)
  public static final Pose2d kSideStartPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));
  public static final Pose2d kNearScaleEmptyPose = new Pose2d(new Translation2d(253.0, 28.0),
      Rotation2d.fromDegrees(10 + 180.0));
  public static final Pose2d kNearScaleFullPose = new Pose2d(new Translation2d(253.0, 28.0 + 5.0),
      Rotation2d.fromDegrees(10.0 + 180.0));

  public static final Pose2d kNearScaleFullPose1 = new Pose2d(new Translation2d(253.0, 28.0 + 8.0),
      Rotation2d.fromDegrees(10.0 + 180.0));

  public static final Pose2d kNearScaleFullPose2 = new Pose2d(new Translation2d(253.0, 28.0 + 11.0 + 7.0),
      Rotation2d.fromDegrees(20.0 + 180.0));

  public static final Pose2d kNearScaleEndPose = new Pose2d(new Translation2d(220.0, 0.0),
      Rotation2d.fromDegrees(45.0));

  public static final Pose2d kFarScaleEmptyPose = new Pose2d(new Translation2d(256.0, 200.0),
      Rotation2d.fromDegrees(-10.0 + 180.0));
  public static final Pose2d kFarScaleFullPose = new Pose2d(new Translation2d(256.0, 200.0 - 5.0),
      Rotation2d.fromDegrees(-15.0 + 180.0));
  public static final Pose2d kFarScaleFullPose1 = new Pose2d(new Translation2d(256.0, 200.0 - 8.0),
      Rotation2d.fromDegrees(-20.0 + 180.0));
  public static final Pose2d kFarScaleFullPose2 = new Pose2d(new Translation2d(256.0, 200.0 - 11.0 - 10.0),
      Rotation2d.fromDegrees(-15.0 + 180.0));

  public static final Pose2d kCenterToIntake = new Pose2d(new Translation2d(-24.0, 0.0), Rotation2d.identity());

  public static final Pose2d kNearCube1Pose = new Pose2d(new Translation2d(183.0, 46.0),
      Rotation2d.fromDegrees(180.0 - 25.0));
  public static final Pose2d kNearCube2Pose = new Pose2d(new Translation2d(180.0, 46.0 + 30.0 + 15.0),
      Rotation2d.fromDegrees(180.0 - 65.0));
  public static final Pose2d kNearCube3Pose = new Pose2d(new Translation2d(170.0, 46.0 + 30.0 * 2 + 20.0),
      Rotation2d.fromDegrees(180.0 - 60.0));

  public static final Pose2d kNearFence1Pose = kNearCube1Pose.transformBy(kCenterToIntake);
  public static final Pose2d kNearFence2Pose = kNearCube2Pose.transformBy(kCenterToIntake);
  public static final Pose2d kNearFence3Pose = kNearCube3Pose.transformBy(kCenterToIntake);

  public static final Pose2d kFarCube1Pose = new Pose2d(new Translation2d(185.0, 180.0),
      Rotation2d.fromDegrees(180.0 + 25.0));
  public static final Pose2d kFarCube2Pose = new Pose2d(new Translation2d(183.0, 180.0 - 30.0 - 15.0),
      Rotation2d.fromDegrees(180.0 + 65.0));
  public static final Pose2d kFarCube3Pose = new Pose2d(new Translation2d(174.0, 180.0 - 30.0 * 2 - 20.0),
      Rotation2d.fromDegrees(180.0 + 60.0));

  public static final Pose2d kFarFence1Pose = kFarCube1Pose.transformBy(kCenterToIntake);
  public static final Pose2d kFarFence2Pose = kFarCube2Pose.transformBy(kCenterToIntake);
  public static final Pose2d kFarFence3Pose = kFarCube3Pose.transformBy(kCenterToIntake);

  // STARTING IN CENTER
  public static final Pose2d kCenterStartPose = new Pose2d(0.0, -4.0, Rotation2d.fromDegrees(180.0));
  public static final Pose2d kSimpleSwitchStartPose = new Pose2d(0.0, -2.0, Rotation2d.fromDegrees(180.0));
  public static final Pose2d kRightSwitchPose = new Pose2d(new Translation2d(100.0, -60.0),
      Rotation2d.fromDegrees(0.0 + 180.0));
  public static final Pose2d kLeftSwitchPose = new Pose2d(new Translation2d(100.0, 60.0),
      Rotation2d.fromDegrees(0.0 + 180.0));

  public static final Pose2d kPyramidCubePose = new Pose2d(new Translation2d(82.0, 5.0),
      Rotation2d.fromDegrees(0.0 + 60.0)).transformBy(kCenterToIntake);
  public static final Pose2d kCenterPyramidCubePose = new Pose2d(new Translation2d(90.0, 0.0),
      Rotation2d.fromDegrees(0.0)).transformBy(kCenterToIntake);
  public static final Pose2d kPyramidCube1Pose = new Pose2d(new Translation2d(106.0, 3.0), Rotation2d.fromDegrees(0.0))
      .transformBy(kCenterToIntake);
  public static final Pose2d kPyramidCube2Pose = new Pose2d(new Translation2d(100.0, 6.0 - 2.0),
      Rotation2d.fromDegrees(0.0 + 60.0)).transformBy(kCenterToIntake);

  public static final Pose2d kSimpleSwitchEndPose = new Pose2d(new Translation2d(160, -106.0),
      Rotation2d.fromDegrees(180.0 + 0.0));

  public static final Pose2d kScalePoseLeft = new Pose2d(new Translation2d(253.0 + 8.0, -84.0),
      Rotation2d.fromDegrees(15.0 + 180.0));
  public static final Pose2d kScalePose1Left = new Pose2d(new Translation2d(253.0 + 8.0, -84.0),
      Rotation2d.fromDegrees(15.0 + 180.0));
  public static final Pose2d kCube1PoseLeft = new Pose2d(new Translation2d(183.0 + 6.0, -84.0 + 18.0),
      Rotation2d.fromDegrees(180.0 - 25.0));
  public static final Pose2d kFence1PoseLeft = kCube1PoseLeft.transformBy(kCenterToIntake);

  public static final Pose2d kScalePoseRight = new Pose2d(new Translation2d(253.0 + 8.0, -96.0),
      Rotation2d.fromDegrees(15.0 + 180.0));
  public static final Pose2d kScalePose1Right = new Pose2d(new Translation2d(253.0 + 8.0, -96.0),
      Rotation2d.fromDegrees(15.0 + 180.0));
  public static final Pose2d kCube1PoseRight = new Pose2d(new Translation2d(183.0 + 6.0, -96.0 + 18.0),
      Rotation2d.fromDegrees(180.0 - 25.0));
  public static final Pose2d kFence1PoseRight = kCube1PoseRight.transformBy(kCenterToIntake);

  public class TrajectorySet {
    public class MirroredTrajectory {
      public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
        this.right = right;
        this.left = TrajectoryUtil.mirrorTimed(right);
      }

      public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
        return left ? this.left : this.right;
      }

      public final Trajectory<TimedState<Pose2dWithCurvature>> left;
      public final Trajectory<TimedState<Pose2dWithCurvature>> right;
    }

    public final MirroredTrajectory sideStartToNearScale;

    private TrajectorySet() {
      sideStartToNearScale = new MirroredTrajectory(getSideStartToNearScale());

    }

    private Trajectory<TimedState<Pose2dWithCurvature>> getSideStartToNearScale() {
      List<Pose2d> waypoints = new ArrayList<>();
      waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0)));
      waypoints.add(new Pose2d(10.0, 0.0, Rotation2d.fromDegrees(180.0)));
      waypoints.add(new Pose2d(-10.0, -4.0, Rotation2d.fromDegrees(180.0)));
    //   waypoints.add(kSideStartPose.transformBy(Pose2d.fromTranslation(new Translation2d(-120.0, 0.0))));
    //   waypoints.add(kNearScaleEmptyPose);
      return generateTrajectory(true, waypoints,
          Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccelElevatorDown)), kMaxVelocity,
          kMaxAccel, kMaxVoltage);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
