/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorMotionMagic extends Command {

  private double target;

  public ElevatorMotionMagic(double target) {
    requires(Robot.elevator);
    this.target = target;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.setTargetPosition(target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.println("T:" + (Robot.elevator.getTargetPosition() + "     ").substring(0, 6) + " V: " + Robot.elevator.getMotorOutputVoltage());
    if(Robot.elevator.isHomed()) {
      Robot.elevator.updateMotionMagic();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(!Robot.elevator.isHomed()) {
      return true;
    }
    return Robot.elevator.isInPosition();
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
