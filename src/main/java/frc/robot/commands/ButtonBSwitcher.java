/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.commands.poses.BallSecond;
import frc.robot.commands.poses.HatchSecond;

public class ButtonBSwitcher extends ConditionalCommand {
  public ButtonBSwitcher() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(new BallSecond(), new HatchSecond());
    // super(Robot.oi.BButton1, Robot.oi.BButton2);
  }

  @Override
  protected boolean condition() {
    return Robot.oi.rotated;
  }
}
