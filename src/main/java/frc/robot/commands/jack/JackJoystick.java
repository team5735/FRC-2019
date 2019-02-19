/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.jack;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class JackJoystick extends Command {

  public JackJoystick() {
    requires(Robot.jack);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.jack.isHomed()) {
      double input = 0;
      input = Robot.oi.subsystemController.rightStick.getYCubed() * 0.5;

      double deltaPosition;
      if (input > 0.07) {
        deltaPosition = input;
      } else if (input < -0.07) {
        deltaPosition = input;
      } else {
        deltaPosition = 0;
      }

      Robot.jack.setTargetPosition(Robot.jack.getTargetPosition() + 0.05 * deltaPosition);
      Robot.jack.updatePosition();

      System.out.println("T:" + (Robot.jack.getTargetPosition() + "     ").substring(0, 6) + " C:"
          + Robot.jack.getSensorPosition() + " PO: "
          + Robot.elevator.getMotorOutputPercent());
    } else {
      System.out.println(Robot.jack.isHomed());
      System.out.println("{JACK} Current Position: " + Robot.jack.getCurrentHeight()
          + " ------- Percent Output: " + Robot.jack.getMotorOutputPercent());
        
      Robot.jack.isLowerLimitSwitchPressed();
      Robot.jack.isUpperLimitSwitchPressed();
      Robot.jack.updatePercentOutput(Robot.oi.subsystemController.rightStick.getYCubed());
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