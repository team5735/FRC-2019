                                                                                                                                                                            /*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.lib.controllers.BobXboxController;
import frc.robot.commands.drivetrain.DrivetrainJoystick;
import frc.robot.commands.elevator.ElevatorMotionMagic;
import frc.robot.commands.elevator.ElevatorResetEncoder;
import frc.robot.commands.hatchholder.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

  public BobXboxController drivetrainController;
  public BobXboxController subsystemController;

  public OI() {
    drivetrainController = new BobXboxController(Constants.DRIVETRAIN_CONTROLLER_USB_PORT);

    subsystemController = new BobXboxController(Constants.SUBSYSTEM_CONTROLLER_USB_PORT);
    subsystemController.aButton.whenPressed(new HatchHolderToggleClaw());
    subsystemController.bButton.whenPressed(new HatchHolderToggleExtentention());
    subsystemController.xButton.whenPressed(new HatchHolderToggleCompressor());
//     subsystemController.aButton.whenPressed(new ElevatorMotionMagic(Robot.elevator.BOTTOM_POSITION));
//     subsystemController.bButton.whenPressed(new ElevatorMotionMagic(Robot.elevator.SECOND_POSITION));
    // subsystemController.xButton.whenPressed(new ElevatorMotionMagic(Robot.elevator.THIRD_POSITION));
    // subsystemController.yButton.whenPressed(new ElevatorMotionMagic(Robot.elevator.MAX_POSITION));

  }
}
