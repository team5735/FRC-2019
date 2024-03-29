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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorJoystick;

public class Elevator extends Subsystem {

  public class Position {
    public static final double ZERO = 0;

    public static final double HATCH_FIRST = 10,
        HATCH_SECOND = 38,
        HATCH_THIRD = 65,
        HATCH_HANDOFF = 5;
    
    public static final double BALL_FIRST = 20, //18
        BALL_SECOND = 44, //42
        BALL_THIRD = 73, //71
        BALL_CARGOSHIP = 30; //30

    private double value;

    public Position(double value) {
      this.value = value;
    }

    public double getValue() {
      return this.value;
    }
  }

  // Subsystem Motors
  private TalonSRX elevatorMotor;
  private VictorSPX elevatorFollowerMotor;

  // Subsystem States
  private boolean isHomed = false;
  private double targetPosition = 0;    // Inches
  private boolean forwardLimitSwitchLastPressed = false;
  private boolean reverseLimitSwitchLastPressed = false;

  // Subsystem Constants
  public static final double THRESHOLD = 1;                  // Inches
  private static final double HEIGHT_LIMIT = 75;              // Inches
  private static final double CRUSING_VEL = 3.5;//7               // Inches / sec
  private static final double TIME_TO_REACH_CRUSING_VEL = 2;  // Sec

  // Encoder Conversion Constants
  private static final double ENCODER_TICKS_PER_REVOLUTION = 4096;
  private static final double GEAR_RATIO = 1 / 10.;
  private static final int SPROCKET_TOOTH_COUNT = 16;
  private static final double LENGTH_OF_LINK = 0.25;
  private static final int NUMBER_OF_STAGES = 3;

  //PID Values
  private final double kP = 0.3 * 1023 / elevatorInchesToEncoderTicks(1);
  private static final double kI = 0;
  private final double kD = 0.6 * 1023 / elevatorInchesToEncoderTicks(1) * 10;
  private static final double kF = 1023. / 12000.;
  private static final double kA = 0; // Arbitrary feed forward (talon directly adds this % out to counteract gravity)

  public Elevator() {
    // Initialize main motor
    elevatorMotor = new TalonSRX(Constants.ELEVATOR_MOTOR_ID);
    elevatorMotor.configFactoryDefault();

    // Configure main motor sensors
    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    elevatorMotor.setInverted(true);
    elevatorMotor.setSensorPhase(true);
    elevatorMotor.overrideLimitSwitchesEnable(true);

    // Set motion magic parameters
    elevatorMotor.configMotionCruiseVelocity(elevatorInchesToEncoderTicks(CRUSING_VEL));
    elevatorMotor.configMotionAcceleration((int)(elevatorInchesToEncoderTicks(CRUSING_VEL * 2)));
    
    // Set main motor PID values
    elevatorMotor.selectProfileSlot(0, 0);
    elevatorMotor.config_kP(0, kP);
    elevatorMotor.config_kI(0, kI);
    elevatorMotor.config_kD(0, kD);
    elevatorMotor.config_kF(0, kF);

    // Initialize follower motor
    elevatorFollowerMotor = new VictorSPX(Constants.ELEVATOR_FOLLOWER_MOTOR_ID);
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

  public void forceSetTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }

  public void updateMotionMagic() {
    isLowerLimitSwitchPressed();
    // System.out.println("Target:" + targetPosition + " Current: " + getCurrentHeight());
    elevatorMotor.set(ControlMode.MotionMagic, elevatorInchesToEncoderTicks(targetPosition));
  }

  public void resetHomed(){
    this.isHomed = false;
  }

  public void updatePercentOutput(double value) {
    isLowerLimitSwitchPressed();
    isUpperLimitSwitchPressed();
    elevatorMotor.set(ControlMode.PercentOutput, value);
  }

  public boolean isInPosition() {
    double positionError = Math.abs(encoderTicksToElevatorInches(getSensorPosition()) - this.targetPosition);
    return positionError < THRESHOLD;
  }

  // public void home() {
  //   elevatorMotor.set(ControlMode.PercentOutput, HOMING_SPEED);
  // }

  public boolean isLowerLimitSwitchPressed() {
    if (elevatorMotor.getSensorCollection().isRevLimitSwitchClosed()) {
      if (!reverseLimitSwitchLastPressed) {
        targetPosition = 0;
        reverseLimitSwitchLastPressed = true;
      }

      isHomed = true;
      elevatorMotor.setSelectedSensorPosition((int)elevatorInchesToEncoderTicks(0), 0, 30);
      return true;
    } else {
      reverseLimitSwitchLastPressed = false;
      return false;
    }
  }

  public boolean isUpperLimitSwitchPressed() {
    return elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public int elevatorInchesToEncoderTicks(double elevatorInches) {
    return (int)((elevatorInches * ENCODER_TICKS_PER_REVOLUTION) / GEAR_RATIO / SPROCKET_TOOTH_COUNT / LENGTH_OF_LINK
    / NUMBER_OF_STAGES);
  }

  public double encoderTicksToElevatorInches(double encoderTicks) {
    return (encoderTicks / ENCODER_TICKS_PER_REVOLUTION) * GEAR_RATIO * SPROCKET_TOOTH_COUNT * LENGTH_OF_LINK
        * NUMBER_OF_STAGES;
  }

  // ===== GETTERS AND SETTERS =====

  public boolean isHomed() {
    return isHomed;
  }

  public void setHomed(boolean homeValue){
    isHomed = homeValue;
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
    return elevatorMotor.getSensorCollection().getQuadraturePosition();
    // return elevatorMotor.getSelectedSensorPosition();
  }

  public double getSensorVelocity() {
    return elevatorMotor.getSelectedSensorVelocity();
  }

  public double getMotorOutputPercent() {
    return elevatorMotor.getMotorOutputPercent();
  }

  public double getErrorInElevatorInches() {
    return encoderTicksToElevatorInches(elevatorMotor.getClosedLoopError());
  }
}
