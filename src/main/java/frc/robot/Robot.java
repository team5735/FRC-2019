/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CargoHolder;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchHolder;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Jack;
import frc.robot.subsystems.SpinnyMotor;
import frc.robot.subsystems.TrajectoryGenerator;

/**
 * FRC team 5735 space mission thing 2019
 * 
 * Written by Walker Whitehouse circa 2019
 */
public class Robot extends TimedRobot {
  public static Drive drive = new Drive();
  public static Elevator elevator = new Elevator();
  public static RobotState robotState = new RobotState();
  public static Jack jack = new Jack();
  public static IntakeArm intakeArm = new IntakeArm();
  public static CargoHolder cargoHolder = new CargoHolder();
  public static HatchHolder hatchHolder = new HatchHolder(); //pneum
  public static SpinnyMotor spinnyMotor = new SpinnyMotor();
  // public static BasicSolenoid basicSolenoid = new BasicSolenoid();
  public static OI oi;

  public static TrajectoryGenerator trajectoryGenerator = new TrajectoryGenerator();

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    oi = new OI();
    trajectoryGenerator.generateTrajectories();
    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    // SmartDashboard.putData("Auto mode", m_chooser);
    SmartDashboard.putBoolean("putInBallMode", oi.rotated);
    SmartDashboard.putBoolean("isElevatorHomed", elevator.isHomed());
    SmartDashboard.putBoolean("isJackHomed", jack.isHomed());
    SmartDashboard.putBoolean("isArmHomed", intakeArm.isHomed());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_chooser.getSelected();

    // /*
    //  * String autoSelected = SmartDashboard.getString("Auto Selector",
    //  * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
    //  * = new MyAutoCommand(); break; case "Default Auto": default:
    //  * autonomousCommand = new ExampleCommand(); break; }
    //  */

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.start();
    // }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Robot.elevator.resetHomed();
    Robot.intakeArm.resetHomed();
    Robot.jack.resetHomed();

    Robot.elevator.forceSetTargetPosition(Robot.elevator.getCurrentHeight());
    Robot.intakeArm.forceSetTargetAngle(Robot.intakeArm.getCurrentDegrees());
    Robot.jack.forceSetTargetPosition(Robot.jack.getCurrentHeight());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    SmartDashboard.putBoolean("isElevatorHomed", elevator.isHomed());
    SmartDashboard.putBoolean("isJackHomed", jack.isHomed());
    SmartDashboard.putBoolean("isArmHomed", intakeArm.isHomed());
    //Periodic testing outputs
    // System.out.println("Drive");
    // System.out.println(drive.periodicOutput());
    // System.out.println("Elevator");
    // System.out.println(elevator.periodicOutput());
    // System.out.println("Intake Arm");
    // System.out.println(intakeArm.periodicOutput());
    // System.out.println("jack");
    // System.out.println(jack.periodicOutput());
      // System.out.println("encoder ticks: " + elevator.getSensorPosition() + "     " + elevator.getSensorVelocity());
      // System.out.println("target" + elevator.getTargetPosition());
      // System.out.println("actual" + elevator.getCurrentHeight());
      // System.out.print("lower limit switch pressed: " + elevator.isLowerLimitSwitchPressed() + "     is upper limit switch pressed: " + elevator.isUpperLimitSwitchPressed());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
