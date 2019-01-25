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
import frc.robot.commands.elevator.ElevatorJoystickCommand;

public class Elevator extends Subsystem {

  private double targetPosition = 0; // inches
  private boolean isHomed = false;
  private TalonSRX elevatorMotor;
  private double homingSpeed = -3;
  private int elevatorThreshold = 2; // Encoder Ticks
  private boolean isHoldingPosition = false;

  private static final double GEAR_RATIO = 4.0 / 3.0; // Gear ratio between motor and Elevator
  private static final int SPROCKET_TOOTH_COUNT = 22;
  private static final double LENGTH_OF_LINK = 0.25;

  private static final int ENCODER_TICKS_PER_REVOLUTION = 4096; // TODO CHECK this?

  private int elevatorFirstSpaceshipPosition = 10; //

  // private ElevatorState state;

  // private enum ElevatorState {
  // MOVING,
  // HOMING,
  // HOLDING,
  // IDLE
  // }

  public Elevator() {
    elevatorMotor = new TalonSRX(Constants.ELEVATOR_MOTOR_ID);
    elevatorMotor.selectProfileSlot(0, 0);
    elevatorMotor.config_kP(0, Constants.elevatorP); //TODO TUNE
    elevatorMotor.config_kI(0, Constants.elevatorI);
    elevatorMotor.config_kD(0, Constants.elevatorD);
    elevatorMotor.config_kF(0, Constants.elevatorF);
    elevatorMotor.configMotionCruiseVelocity(1500);
    elevatorMotor.configMotionAcceleration(1500);
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    elevatorMotor.setInverted(true);
    elevatorMotor.setSensorPhase(false);
    // state = ElevatorState.IDLE;
    elevatorMotor.overrideLimitSwitchesEnable(true);
    resetSensorPosition();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ElevatorJoystickCommand());
  }

  public void resetSensorPosition() {
    elevatorMotor.setSelectedSensorPosition(0, 0, 30);
  }

  public int getElevatorFirstSpaceshipPosition() {
    return elevatorFirstSpaceshipPosition;
  }

  // inches
  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }

  public double nativeunitspersecond() {
    return elevatorMotor.getSelectedSensorVelocity();
  }

  public void moveToPosition() {
    // elevatorMotor.set(ControlMode.MotionMagic, elevatorInchesToEncoderTicks(targetPosition));
    elevatorMotor.set(ControlMode.MotionMagic, targetPosition * 4096);
  }

  public void percentOutput(double percent) {
    elevatorMotor.set(ControlMode.PercentOutput, percent);
  }

  public void home() {
    elevatorMotor.set(ControlMode.Velocity, homingSpeed);
  }

  public boolean isLowerLimitPressed() {
    return elevatorMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean isUppperLimitPressed() {
    return elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public double elevatorInchesToEncoderTicks(double elevatorInches) {
    return (elevatorInches / (GEAR_RATIO * SPROCKET_TOOTH_COUNT * LENGTH_OF_LINK)) * ENCODER_TICKS_PER_REVOLUTION / 2;
  }

  public int encoderTicksToElevatorInches(double encoderTicks) {
    return (int)((encoderTicks / ENCODER_TICKS_PER_REVOLUTION) * GEAR_RATIO * SPROCKET_TOOTH_COUNT * LENGTH_OF_LINK * 2);
  }

  public double getTargetPositionInEncoderTicks() {
    return elevatorInchesToEncoderTicks(targetPosition);
  }

  public double getSensorPositionInElevatorInches() {
    return encoderTicksToElevatorInches(getSensorPosition());
  }

  public void setHomed(boolean isHomed) {
    this.isHomed = isHomed;
  }

  public boolean isHomed() {
    return isHomed;
  }

  public void setIsHoldingPosition(boolean isHoldingPosition) {
    this.isHoldingPosition = isHoldingPosition;
  }

  public boolean isHoldingPosition() {
    return isHoldingPosition;
  }

  public void setSensorValue(int sensorPos) {
    elevatorMotor.setSelectedSensorPosition(sensorPos);
  }

  public int getSensorPosition() {
    return elevatorMotor.getSelectedSensorPosition();
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public boolean isInPosition(int targetPosition) {
    int currentPosition = this.getSensorPosition();
    int positionError = Math.abs(currentPosition - targetPosition);
    if (positionError < elevatorThreshold) {
      return true;
    } else {
      return false;
    }
  }
}
