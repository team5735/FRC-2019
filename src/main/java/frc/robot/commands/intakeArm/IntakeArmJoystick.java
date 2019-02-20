/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intakeArm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeArmJoystick extends Command {
  public IntakeArmJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.intakeArm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.intakeArm.isHomed()) {
      // System.out.println("{INTAKE} Target: " + Robot.intakeArm.getTargetDegress() + "Current Degrees: " + Robot.intakeArm.getCurrentDegrees() + " ------- PO: " + Robot.intakeArm.getPercentOutput());
      // System.out.println("{INTAKE} Target: " + Robot.intakeArm.getTargetDegress() + "Current Height: " + Robot.intakeArm.degreesToInches(Robot.intakeArm.getCurrentDegrees()) + " ------- PO: " + Robot.intakeArm.getPercentOutput());

      Robot.intakeArm.setTargetAngle(Robot.intakeArm.getTargetDegress() - 2 * Robot.oi.subsystemController.leftStick.getYCubed());
      // Robot.intakeArm.setTargetAngle(Robot.intakeArm.inchesToDegrees(10 * Math.abs(Robot.oi.subsystemController.leftStick.getYCubed())));
      Robot.intakeArm.updateMotionMagic();
    } else {
      // System.out.println("{INTAKE} Current Degrees: " + Robot.intakeArm.getCurrentDegrees() + " ------- Speed: " + Robot.intakeArm.getArmVelocityInEncoderTicks());

      Robot.intakeArm.isLowerLimitSwitchPressed();
      Robot.intakeArm.isUpperLimitSwitchPressed();
      Robot.intakeArm.updatePercentOutputOnArm(-Robot.oi.subsystemController.leftStick.getYCubed());
    }    
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
