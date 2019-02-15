/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class IntakeArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX intakeArmMotor;
  private TalonSRX intakeArmFollower;

  // PID Values
  private static final double kP = 0.1;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kF = 0;
  private static final double kA = 0; // Arbitrary feed forward (talon directly adds this % out to counteract gravity)

  public IntakeArm() {
    intakeArmMotor = new TalonSRX(Constants.INTAKE_ARM_MOTOR_ID);
    intakeArmMotor.configFactoryDefault();

    intakeArmMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    intakeArmMotor.setInverted(true);
    intakeArmMotor.setSensorPhase(true);
    intakeArmMotor.overrideLimitSwitchesEnable(true);
    resetSensorPosition();

    // Set motion magic parameters
    // intakeArmMotor.configMotionCruiseVelocity(elevatorInchesToEncoderTicks(CRUSING_VEL));
    // intakeArmMotor.configMotionAcceleration((int) (elevatorInchesToEncoderTicks(CRUSING_VEL) / TIME_TO_REACH_CRUSING_VEL));

    // Set main motor PID values
    intakeArmMotor.selectProfileSlot(0, 0);
    intakeArmMotor.config_kP(0, kP);
    intakeArmMotor.config_kI(0, kI);
    intakeArmMotor.config_kD(0, kD);
    intakeArmMotor.config_kF(0, kF);

    intakeArmFollower = new TalonSRX(Constants.INTAKE_ARM_MOTOR_FOLLOWER_ID);
    intakeArmFollower.configFactoryDefault();

    intakeArmFollower.follow(intakeArmMotor);
    intakeArmFollower.setInverted(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeArmJoystick());
  }

  public void updatePercentOutput(double value) {
    intakeArmMotor.set(ControlMode.PercentOutput, value);
  }

  public void resetSensorPosition() {
    intakeArmMotor.setSelectedSensorPosition(0, 0, 30);
  }
}
