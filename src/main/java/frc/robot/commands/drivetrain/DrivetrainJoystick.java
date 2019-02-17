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
    System.out.println("foo");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("foo");
    Robot.drive.cheesyDrive(Robot.oi.drivetrainController.rightStick.getYCubed(),
        Robot.oi.drivetrainController.leftStick.getXCubed() * 0.5,
         Robot.oi.drivetrainController.triggers.getLeft() > 0.5); // left trigger 
    // Robot.drive.drive(ControlMode.Velocity,
    // Robot.oi.drivetrainController.leftStick.getYCubed(),
    // Robot.oi.drivetrainController.rightStick.getYCubed());
    // Robot.drivetrain.updateVelocityPercent(Robot.oi.drivetrainController.leftStick.getYCubed(),
    // Robot.oi.drivetrainController.rightStick.getYCubed());
    // Robot.drivetrain.updateArcadePercent(Robot.oi.drivetrainController.rightStick.getYCubed(),
    // Robot.oi.drivetrainController.leftStick.getXCubed(), false);
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
