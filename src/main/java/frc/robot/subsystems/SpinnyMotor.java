/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.BallIntakeRun;

/**
 * Add your docs here.
 */
public class SpinnyMotor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX spinnyMotor;

  public SpinnyMotor() {
    spinnyMotor = new TalonSRX(Constants.INTAKE_ARM_SPINNER_MOTOR_ID);
    spinnyMotor.configFactoryDefault();
  }

  public void updatePercentOutputOnSpinner(double value) {
    spinnyMotor.set(ControlMode.PercentOutput, value);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new BallIntakeRun());
  }
}
