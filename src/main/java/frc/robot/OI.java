/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.Constants;
import frc.robot.commands.BasicSolenoidToggleCompressor;
import frc.robot.commands.BasicSolenoidToggleSolenoid;
import frc.robot.commands.elevator.ElevatorGoToFirstSpaceship;
import frc.robot.commands.elevator.ElevatorResetEncoder;


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

  public XboxController drivetrainXboxController;
  Button drivetrainXboxA, drivetrainXboxB, drivetrainXboxC, drivetrainXboxD;

  public XboxController subsystemXboxController;
  Button subsystemXboxA, subsystemXboxB, subsystemXboxC, subsystemXboxD;

  public OI() {
    drivetrainXboxController = new XboxController(Constants.DRIVETRAIN_XBOX_CONTROLLER_ID);
    subsystemXboxController = new XboxController(Constants.SUBSYSTEM_XBOX_CONTROLLER_ID);

    drivetrainXboxA = new JoystickButton(drivetrainXboxController, Constants.XBOX_A_BUTTON_ID);
    drivetrainXboxB = new JoystickButton(drivetrainXboxController, Constants.XBOX_B_BUTTON_ID);
    drivetrainXboxC = new JoystickButton(drivetrainXboxController, Constants.XBOX_C_BUTTON_ID);
    drivetrainXboxD = new JoystickButton(drivetrainXboxController, Constants.XBOX_D_BUTTON_ID);
    
    /* 
    drivetrainXboxA.whenPressed();
    drivetrainXboxB.whenPressed();
    drivetrainXboxC.whenPressed();
    drivetrainXboxD.whenPressed();
    */

    subsystemXboxA = new JoystickButton(subsystemXboxController, Constants.XBOX_A_BUTTON_ID);
    subsystemXboxB = new JoystickButton(subsystemXboxController, Constants.XBOX_B_BUTTON_ID);
    subsystemXboxC = new JoystickButton(subsystemXboxController, Constants.XBOX_C_BUTTON_ID);
    subsystemXboxD = new JoystickButton(subsystemXboxController, Constants.XBOX_D_BUTTON_ID);

    // subsystemXboxA.whenPressed(new ElevatorGoToFirstSpaceship());
    subsystemXboxB.whenPressed(new ElevatorResetEncoder());
    // subsystemXboxC.whenPressed(new BasicSolenoidToggleSolenoid());
    // subsystemXboxD.whenPressed(new BasicSolenoidToggleCompressor());
  }
}
