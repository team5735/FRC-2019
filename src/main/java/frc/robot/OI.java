/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.controllers.BobXboxController;
import frc.robot.commands.ButtonASwitcher;
import frc.robot.commands.ButtonBSwitcher;
import frc.robot.commands.ButtonXSwitcher;
import frc.robot.commands.ButtonYSwitcher;
import frc.robot.commands.RotateOI;
import frc.robot.commands.combos.DrivetrainControllerCancel;
import frc.robot.commands.combos.JackIntakeArmClimbDrivetrainClimb;
import frc.robot.commands.combos.SubsystemControllerCancel;
import frc.robot.commands.elevator.ElevatorResetHome;
import frc.robot.commands.hatchholder.HatchHolderClawClosed;
import frc.robot.commands.hatchholder.HatchHolderClawOpen;
import frc.robot.commands.hatchholder.HatchHolderExtenderClosed;
import frc.robot.commands.hatchholder.HatchHolderExtenderOpen;
import frc.robot.commands.hatchholder.HatchHolderToggleCompressor;
import frc.robot.commands.intakeArm.IntakeArmResetHome;
import frc.robot.commands.poses.BallFirst;
import frc.robot.commands.poses.BallSecond;
import frc.robot.commands.poses.BallShip;
import frc.robot.commands.poses.BallThird;
import frc.robot.commands.poses.HatchFirst;
import frc.robot.commands.poses.HatchSecond;
import frc.robot.commands.poses.HatchThird;
import frc.robot.commands.poses.IntakePose;
import frc.robot.commands.poses.ReadyPose;
import frc.robot.commands.poses.StartingPose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeArm;

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

  public boolean rotated = false;

  public Command AButton1 = new BallFirst();
  public Command AButton2 = new HatchFirst();
  public Command BButton1 = new BallSecond();
  public Command BButton2 = new HatchSecond();
  public Command XButton1 = new IntakePose();
  public Command XButton2 = new StartingPose();
  public Command YButton1 = new BallThird();
  public Command YButton2 = new HatchThird();

  public OI() {
    drivetrainController = new BobXboxController(Constants.DRIVETRAIN_CONTROLLER_USB_PORT);
    drivetrainController.xButton.whenPressed(new ReadyPose());
    drivetrainController.startButton.whenPressed(new JackIntakeArmClimbDrivetrainClimb());

    drivetrainController.selectButton.whenPressed(new DrivetrainControllerCancel());

    drivetrainController.Dpad.Up.whenPressed(new HatchHolderToggleCompressor());

    SmartDashboard.putBoolean("putInBallMode", rotated);

    subsystemController = new BobXboxController(Constants.SUBSYSTEM_CONTROLLER_USB_PORT);
    subsystemController.Dpad.Up.whenPressed(new HatchHolderExtenderOpen());
    subsystemController.Dpad.Down.whenPressed(new HatchHolderExtenderClosed());
    subsystemController.Dpad.Left.whenPressed(new HatchHolderClawOpen());
    subsystemController.Dpad.Right.whenPressed(new HatchHolderClawClosed());

    subsystemController.selectButton.whenPressed(new SubsystemControllerCancel());

    subsystemController.aButton.whenPressed(new ButtonASwitcher());
    subsystemController.bButton.whenPressed(new ButtonBSwitcher());
    subsystemController.xButton.whenPressed(new ButtonXSwitcher());
    subsystemController.yButton.whenPressed(new ButtonYSwitcher());

    subsystemController.leftStickButton.whenPressed(new IntakeArmResetHome());
    subsystemController.rightStickButton.whenPressed(new ElevatorResetHome());

    subsystemController.startButton.whenReleased(new RotateOI());
  }

  public void rotate() {
    rotated = !rotated;
    SmartDashboard.putBoolean("putInBallMode", rotated);
  }

}
