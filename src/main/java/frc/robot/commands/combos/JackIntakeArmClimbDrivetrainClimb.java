/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.combos;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Jack;

public class JackIntakeArmClimbDrivetrainClimb extends Command {

  private double position; // Inches from ready position
  private double up; // Inches up jack (also negative)

  private double velocity = 0.1; // inches per 50 ms
  private double adjustmentspeed = 0.1;

  private double jackadjustment;
  private double intakearmadjustment;

  public JackIntakeArmClimbDrivetrainClimb() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.jack);
    requires(Robot.intakeArm);
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    position = 0;
    jackadjustment = 0;
    intakearmadjustment = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    intakearmadjustment += Robot.oi.drivetrainController.triggers.getLeft() * 0.1;
    jackadjustment += Robot.oi.drivetrainController.triggers.getRight() * 0.1;
    if (Robot.oi.drivetrainController.aButton.get()) {
      if (up < 0) {
        up += velocity;
      } else if (position < (Constants.HEIGHT_OF_BOX + Math.abs(intakearmadjustment - jackadjustment))) {
        position += velocity;
      } else {
        Robot.oi.drivetrainController.setRumble(RumbleType.kLeftRumble, 1);
        Robot.oi.drivetrainController.setRumble(RumbleType.kRightRumble, 1);
      }
    } else if (Robot.oi.drivetrainController.bButton.get()) {
      up -= velocity * 0.5;
    }
    // Robot.jack.setTargetPosition(Jack.JACK_READY_POSITION + position + jackadjustment - up);
    // Robot.intakeArm.setTargetAngle(IntakeArm.Angle.READY + Robot.intakeArm.inchesToDegrees(-position - intakearmadjustment + up));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
