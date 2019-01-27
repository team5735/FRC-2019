/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorJoystickCommand extends Command {

  private static final int encoderIncrement = 100;

  public ElevatorJoystickCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Robot.elevator.setTargetPosition(0);
    Robot.elevator.setTargetPosition(1.0 * Robot.oi.leftJoy.getY(Hand.kLeft));
    // Robot.elevator.percentOutput(Robot.oi.leftJoy.getY(Hand.kLeft));
    // Robot.elevator.setTargetPosition(targetPosition);
    // System.out.println(Robot.oi.leftJoy.getY(Hand.kLeft));
    // System.out.println("Sensor: " + Robot.elevator.getSensorPosition());
    // System.out.println("Target: " + Robot.elevator.getTargetPosition() * 4096);
    // System.out.println(Robot.elevator.nativeunitspersecond());
    Robot.elevator.moveToPosition();
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
