/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakeArm.unused;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeArmPosition extends Command {

  private double target;

  public IntakeArmPosition(double target) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intakeArm);
    this.target = target;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.intakeArm.setTargetAngle(target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.println("{INTAKE} Target: " + Robot.intakeArm.getTargetDegress() + "Current Degrees: " + Robot.intakeArm.getCurrentDegrees() + " ------- PO: " + Robot.intakeArm.getPercentOutput());

    Robot.intakeArm.updateMotionMagic();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
