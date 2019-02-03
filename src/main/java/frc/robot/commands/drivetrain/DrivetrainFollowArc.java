/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.trajectory.TimedView;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.TimedState;
import frc.robot.Robot;

public class DrivetrainFollowArc extends Command {
  private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory;
  private final boolean resetPose;

  public DrivetrainFollowArc(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
    this(trajectory, false);
  }

  public DrivetrainFollowArc(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
    this.trajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
    this.resetPose = resetPose;
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Starting trajectory! (length=" + trajectory.getRemainingProgress() + ")");
    if (resetPose) {
      Robot.robotState.reset(Timer.getFPGATimestamp(), trajectory.getState().state().getPose());
    }
    Robot.drive.setTrajectory(trajectory);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.drive.isDoneWithTrajectory()) {
      System.out.println("Trajectory finished");
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
