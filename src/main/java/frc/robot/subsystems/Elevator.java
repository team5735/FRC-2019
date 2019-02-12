/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorJoystick;
import frc.robot.commands.elevator.ElevatorMotionMagic;

public class Elevator extends Subsystem {
  // Subsystem Setpoints
  public final double BOTTOM_POSITION = 0,
    FIRST_POSITION = 10,
    SECOND_POSITION = 30,
    THIRD_POSITION = 50,
    MAX_POSITION = 70;

  // Subsystem Motors
  private TalonSRX elevatorMotor, elevatorFollowerMotor;

  // Subsystem States
  private boolean isHomed = false;
  private double targetPosition = 0;    // Inches
  private double elevatorFirstSpaceshipPosition = 10;
  private double elevatorSecondSpaceshipPosition = 20;

  // Subsystem Constants
  private static final double HOMING_SPEED = -0.2;            // Percent Output
  private static final double THRESHOLD = 1;                  // Inches
  private static final double HEIGHT_LIMIT = 80;              // Inches
  private static final double CRUSING_VEL = 50;               // Inches / sec
  private static final double TIME_TO_REACH_CRUSING_VEL = 2;  // Sec

  // Encoder Conversion Constants
  private static final double ENCODER_TICKS_PER_REVOLUTION = 12;
  private static final double GEAR_RATIO = 1 / 70.;
  private static final int SPROCKET_TOOTH_COUNT = 16;
  private static final double LENGTH_OF_LINK = 0.25;
  private static final int NUMBER_OF_STAGES = 3;

  //PID Values
  private static final double kP = 3;
  private static final double kI = 0;
  private static final double kD = 5;
  private static final double kF = 0.337 * 1023 / 70.;
  private static final double kA = 0; // Arbitrary feed forward (talon directly adds this % out to counteract gravity)

  public Elevator() {
    // Initialize main motor
    elevatorMotor = new TalonSRX(Constants.ELEVATOR_MOTOR_ID);
    elevatorMotor.configFactoryDefault();

    // Configure main motor sensors
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    elevatorMotor.setInverted(true);
    elevatorMotor.setSensorPhase(true);
    elevatorMotor.overrideLimitSwitchesEnable(true);
    resetSensorPosition();

    // Set motion magic parameters
    elevatorMotor.configMotionCruiseVelocity(elevatorInchesToEncoderTicks(CRUSING_VEL));
    elevatorMotor.configMotionAcceleration((int)(elevatorInchesToEncoderTicks(CRUSING_VEL)/TIME_TO_REACH_CRUSING_VEL));
    
    // Set main motor PID values
    elevatorMotor.selectProfileSlot(0, 0);
    elevatorMotor.config_kP(0, kP);
    elevatorMotor.config_kI(0, kI);
    elevatorMotor.config_kD(0, kD);
    elevatorMotor.config_kF(0, kF);

    // Initialize follower motor
    elevatorFollowerMotor = new TalonSRX(Constants.ELEVATOR_FOLLOWER_MOTOR_ID);
    elevatorFollowerMotor.configFactoryDefault();
    // elevatorFollowerMotor.set(ControlMode.Follower, Constants.ELEVATOR_MOTOR_ID);
    elevatorFollowerMotor.follow(elevatorMotor);
    elevatorFollowerMotor.setInverted(false);
  }

  @Override
  /**
   * This command will run whenever there is no other command using the subsystem
   */
  public void initDefaultCommand() {
    setDefaultCommand(new ElevatorJoystick());
  }

  /**
   * Sets target position (in inches) for motion magic control
   */
  public void setTargetPosition(double targetPosition) {
    if (targetPosition < 0) {
      this.targetPosition = 0;
    } else if (targetPosition > HEIGHT_LIMIT) {
      this.targetPosition = HEIGHT_LIMIT;
    }else{
      this.targetPosition = targetPosition;
    }
  }

  public double getElevatorFirstSpaceshipPosition() {
    return elevatorFirstSpaceshipPosition;
  }

  public double getElevatorSecondSpaceshipPosition() {
    return elevatorSecondSpaceshipPosition;
  }

  public void updateMotionMagic() {
    elevatorMotor.set(ControlMode.MotionMagic, elevatorInchesToEncoderTicks(targetPosition), DemandType.ArbitraryFeedForward, kA);
  }

  public void updatePercentOutput(double value) {
    elevatorMotor.set(ControlMode.PercentOutput, value);
  }

  public boolean isInPosition() {
    double positionError = Math.abs(encoderTicksToElevatorInches(getSensorPosition()) - this.targetPosition);
    return positionError < THRESHOLD;
  }

  public void home() {
    elevatorMotor.set(ControlMode.PercentOutput, HOMING_SPEED);
  }

  public void resetSensorPosition() {
    elevatorMotor.setSelectedSensorPosition(0, 0, 30);
  }

  public boolean isLowerLimitPressed() {
    if (elevatorMotor.getSensorCollection().isRevLimitSwitchClosed()) {
      resetSensorPosition();
      return true;
    } else {
      return false;
    }
  }

  public boolean isUpperLimitPressed() {
    return elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  private static int elevatorInchesToEncoderTicks(double elevatorInches) {
    return (int)((elevatorInches * ENCODER_TICKS_PER_REVOLUTION) / GEAR_RATIO / SPROCKET_TOOTH_COUNT / LENGTH_OF_LINK
    / NUMBER_OF_STAGES);
  }

  private static double encoderTicksToElevatorInches(double encoderTicks) {
    return (encoderTicks / ENCODER_TICKS_PER_REVOLUTION) * GEAR_RATIO * SPROCKET_TOOTH_COUNT * LENGTH_OF_LINK
        * NUMBER_OF_STAGES;
  }

  // ===== GETTERS AND SETTERS =====

  public void setHomed(boolean isHomed) {
    this.isHomed = isHomed;
  }

  public boolean isHomed() {
    return isHomed;
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public double getCurrentHeight() {
    return encoderTicksToElevatorInches(getSensorPosition());
  }

  public void setSensorValue(int sensorPos) {
    elevatorMotor.setSelectedSensorPosition(sensorPos);
  }

  public int getSensorPosition() {
    elevatorMotor.set(ControlMode.PercentOutput, 0.5);
    return elevatorMotor.getSensorCollection().getQuadraturePosition();
    // return elevatorMotor.getSelectedSensorPosition();
  }

  public double getSensorVelocity() {
    return elevatorMotor.getSelectedSensorVelocity();
  }

  public double getMotorOutputPercent() {
    return elevatorMotor.getMotorOutputPercent();
  }
}
