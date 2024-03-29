/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakeArm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeArmMotionMagic extends Command {

  private double targetAngle;

  public IntakeArmMotionMagic(double targetAngle) {
    requires(Robot.intakeArm);
    this.targetAngle = targetAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.intakeArm.setTargetAngle(targetAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.intakeArm.isHomed()) {
      Robot.intakeArm.updateMotionMagic();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(!Robot.intakeArm.isHomed()) {
      return true;
    }
    return Robot.intakeArm.isInPosition();
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
