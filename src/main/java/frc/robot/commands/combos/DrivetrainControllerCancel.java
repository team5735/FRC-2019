/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.combos;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drivetrain.CancelDrivetrain;
import frc.robot.commands.jack.CancelJack;

public class DrivetrainControllerCancel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DrivetrainControllerCancel() {
    addParallel(new CancelDrivetrain());
    addSequential(new CancelJack());
  }
}
