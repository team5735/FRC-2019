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
import frc.robot.commands.jack.JackJoystick;

/**
 * Add your docs here.
 */
public class Jack extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX jackMotor;

  public Jack() {
    jackMotor = new TalonSRX(Constants.JACK_MOTOR_ID);
    jackMotor.configFactoryDefault();

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new JackJoystick());
  }

  // public void updateMotionMagic() {
  // jackMotor.set(ControlMode.MotionMagic,
  // elevatorInchesToEncoderTicks(targetPosition),
  // DemandType.ArbitraryFeedForward, kA);
  // }

  public void updatePercentOutput(double value) {
    jackMotor.set(ControlMode.PercentOutput, value);
  }

  public void resetSensorPosition() {
    jackMotor.setSelectedSensorPosition(0, 0, 30);
  }
}
