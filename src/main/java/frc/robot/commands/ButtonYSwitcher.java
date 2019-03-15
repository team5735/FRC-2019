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
import frc.robot.commands.poses.BallThird;
import frc.robot.commands.poses.HatchThird;

public class ButtonYSwitcher extends ConditionalCommand {
  public ButtonYSwitcher() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    super(new BallThird(), new HatchThird());
    // super(Robot.oi.YButton1, Robot.oi.YButton2);
  }

  @Override
  protected boolean condition() {
    return Robot.oi.rotated;
  }
}
