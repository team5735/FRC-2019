/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.command.Command;
import frc.lib.controllers.BobXboxController;
import frc.lib.util.DriveSignal;
import frc.robot.OI;
import frc.robot.Robot;

public class DrivetrainJoystick extends Command {
  public DrivetrainJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double power = 0.7;
    if (Robot.oi.drivetrainController.getRawButton(7)) {
      power = 0.99;
    }

    if (Robot.oi.drivetrainController.leftBumper.get()) {
      Robot.drive.cheesyDrive(-Robot.oi.drivetrainController.rightStick.getYCubedWithDeadband(0.05) * 0.5,
          -Robot.oi.drivetrainController.leftStick.getXCubedWithDeadband(0.05) * power,
      Robot.oi.drivetrainController.rightBumper.get()); // left trigger
    } else {
      Robot.drive.cheesyDrive(Robot.oi.drivetrainController.rightStick.getYCubedWithDeadband(0.05) * 0.5,
          -Robot.oi.drivetrainController.leftStick.getXCubedWithDeadband(0.05) * power,
      Robot.oi.drivetrainController.rightBumper.get()); // left trigger
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
