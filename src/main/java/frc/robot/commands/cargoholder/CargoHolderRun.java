/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargoholder;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
// ====================================================================================
// ======================= CHANGE THIS LATER WHEN YOU HAVE TIME!!! ====================
// ====================================================================================

public class CargoHolderRun extends Command {
  public CargoHolderRun() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.cargoHolder);
  }

  private double speed = 0.4;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.subsystemController.Dpad.Up.get()) {
      Robot.cargoHolder.run(speed);
    } else if (Robot.oi.subsystemController.Dpad.Down.get()) {
      Robot.cargoHolder.run(-speed);
    } else {
      Robot.cargoHolder.run(0);
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
