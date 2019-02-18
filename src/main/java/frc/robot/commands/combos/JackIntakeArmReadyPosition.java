/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.combos;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ControllerRumble;
import frc.robot.commands.intakeArm.IntakeArmPosition;
import frc.robot.commands.jack.JackPosition;
import frc.robot.subsystems.IntakeArm;

public class JackIntakeArmReadyPosition extends CommandGroup {
  /**
   * Add your docs here.
   */
  public JackIntakeArmReadyPosition() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    // addParallel(new ElevatorArm(0, IntakeArm.Angle.READY));
    // addSequential(new JackPosition(Constants.JACK_READY_POSITION));
    // addSequential(new ControllerRumble());
  }
}
