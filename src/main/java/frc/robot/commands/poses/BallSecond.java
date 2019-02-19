/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.poses;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorMotionMagic;
import frc.robot.commands.intakeArm.IntakeArmMotionMagic;
import frc.robot.commands.intakeArm.IntakeArmPreventCollsion;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeArm;

public class BallSecond extends CommandGroup {
  /**
   * Add your docs here.
   */
  public BallSecond() {
    addSequential(new IntakeArmPreventCollsion());
    addSequential(new ElevatorMotionMagic(Elevator.Position.BALL_SECOND));
    // addSequential(new IntakeArmMotionMagic(IntakeArm.Angle.INSIDE));
  }
}
