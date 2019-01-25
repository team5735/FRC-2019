/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorGoToFirstSpaceshipCommand extends Command {

  private int elevatorFirstSpaceshipPosition = Robot.elevator.getElevatorFirstSpaceshipPosition();

  public ElevatorGoToFirstSpaceshipCommand() {
    requires(Robot.elevator);
  }

  @Override
  protected void initialize() {
    Robot.elevator.setTargetPosition(elevatorFirstSpaceshipPosition);
    System.out.println("mobing to spaceshpi");
  }

  @Override
  protected void execute() {
    System.out.println("Sensor: " + Robot.elevator.getSensorPosition());
    System.out.println("Target: " + Robot.elevator.getTargetPosition() * 4096);
    Robot.elevator.moveToPosition();
  }

  @Override
  protected boolean isFinished() {
    return Robot.elevator.isInPosition(elevatorFirstSpaceshipPosition);
  }

  @Override
  protected void end() {
    System.out.println("stopping moving to speaceship");
  }

  @Override
  protected void interrupted() {
  }
}
