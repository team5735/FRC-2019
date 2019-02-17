/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.cargoholder.CargoHolderRun;

/**
 * Add your docs here.
 */
public class CargoHolder extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private VictorSPX cargoIntakeMotor;
  private VictorSPX cargoIntakeMotorFollower;

  public CargoHolder() {
    cargoIntakeMotor = new VictorSPX(Constants.CARGO_INTAKE_MOTOR_ID);
    // cargoIntakeMotor = new TalonSRX(Constants.CARGO_INTAKE_MOTOR_ID);
    cargoIntakeMotor.configFactoryDefault();
    // cargoIntakeMotor.overrideLimitSwitchesEnable(true);

    cargoIntakeMotorFollower = new VictorSPX(Constants.CARGO_INTAKE_MOTOR_FOLLOWER_ID);
    cargoIntakeMotorFollower.configFactoryDefault();
    // cargoIntakeMotorFollower.overrideLimitSwitchesEnable(true);
    cargoIntakeMotorFollower.follow(cargoIntakeMotor);
    cargoIntakeMotorFollower.setInverted(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new CargoHolderRun());
  }

  public void run(double targetPercent) {
    cargoIntakeMotor.set(ControlMode.PercentOutput, targetPercent);
  }

  public String periodicOutput() {
    return "";
  }
}
