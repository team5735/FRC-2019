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
import frc.robot.commands.jack.JackHoldPosition;
import frc.robot.commands.jack.JackJoystick;

/**
 * Add your docs here.
 */
public class Jack extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX jackMotor;

  private double targetPosition = 0;

  // Subsystem Constants
  private static final double THRESHOLD = 1;                  // Inches
  private static final double HEIGHT_LIMIT = 15;              // Inches
  private static final double CRUSING_VEL = 50;               // Inches / sec
  private static final double TIME_TO_REACH_CRUSING_VEL = 2;  // Sec

  // Encoder Conversion Constants
  private static final double ENCODER_TICKS_PER_REVOLUTION = 12;
  private static final double GEAR_RATIO = 1. / 100. * 18. / 30.;
  private static final int SPROCKET_TOOTH_COUNT = 22;
  private static final double LENGTH_OF_LINK = 0.25;

  public static final double JACK_READY_POSITION = 2;

  private boolean isHomed = false;

  private boolean forwardLimitSwitchLastPressed = false;
  private boolean reverseLimitSwitchLastPressed = false;

  // PID Values
  private static final double kP = 1;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kF = 0;
  private static final double kA = 0; // Arbitrary feed forward (talon directly adds this % out to counteract gravity)

  public Jack() {
    jackMotor = new TalonSRX(Constants.JACK_MOTOR_ID);
    jackMotor.configFactoryDefault();

    jackMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    jackMotor.setInverted(false);
    jackMotor.setSensorPhase(true);
    jackMotor.overrideLimitSwitchesEnable(true);
    resetSensorPosition();

    // Set motion magic parameters
    // jackMotor.configMotionCruiseVelocity(jackInchesToEncoderTicks(CRUSING_VEL));
    // jackMotor.configMotionAcceleration((int)(jackInchesToEncoderTicks(CRUSING_VEL) / TIME_TO_REACH_CRUSING_VEL));

    // Set main motor PID values
    jackMotor.selectProfileSlot(0, 0);
    jackMotor.config_kP(0, kP);
    jackMotor.config_kI(0, kI);
    jackMotor.config_kD(0, kD);
    jackMotor.config_kF(0, kF);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    if (Constants.JACK_DO_STUFF) {
      setDefaultCommand(new JackJoystick());
    }
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

  public boolean isInPosition() {
    double positionError = Math.abs(encoderTicksToJackInches(getSensorPosition()) - this.targetPosition);
    return positionError < THRESHOLD;
  }

  private static int jackInchesToEncoderTicks(double jackInches) {
    return (int)((jackInches * ENCODER_TICKS_PER_REVOLUTION) / GEAR_RATIO / SPROCKET_TOOTH_COUNT / LENGTH_OF_LINK);
  }

  private static double encoderTicksToJackInches(double encoderTicks) {
    return (encoderTicks / ENCODER_TICKS_PER_REVOLUTION) * GEAR_RATIO * SPROCKET_TOOTH_COUNT * LENGTH_OF_LINK;
  }

  public void updatePosition() {
    System.out.println("{JACK} Target: " + getTargetPosition() + "Current Height: "+ getCurrentHeight() + " ------- PO: " + getMotorOutputPercent());
    jackMotor.set(ControlMode.Position, jackInchesToEncoderTicks(targetPosition),
        DemandType.ArbitraryFeedForward, kA);
    // jackMotor.set(ControlMode.Position, jackInchesToEncoderTicks(targetPosition));
  }

  public void updatePercentOutput(double value) {
    jackMotor.set(ControlMode.PercentOutput, value);
  }

  public void resetSensorPosition() {
    jackMotor.setSelectedSensorPosition(0, 0, 30);
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public double getCurrentHeight() {
    return encoderTicksToJackInches(getSensorPosition());
  }

  public void setSensorValue(int sensorPos) {
    jackMotor.setSelectedSensorPosition(sensorPos);
  }

  public int getSensorPosition() {
    return jackMotor.getSensorCollection().getQuadraturePosition();
    // return jackMotor.getSelectedSensorPosition();
  }

  public double getSensorVelocity() {
    return jackMotor.getSelectedSensorVelocity();
  }

  public double getMotorOutputPercent() {
    return jackMotor.getMotorOutputPercent();
  }

  public boolean isUpperLimitSwitchPressed() {
    return jackMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean isLowerLimitSwitchPressed() {
    if (jackMotor.getSensorCollection().isRevLimitSwitchClosed()) {
      if (!reverseLimitSwitchLastPressed) {
        targetPosition = 0;
        reverseLimitSwitchLastPressed = true;
      }

      isHomed = true;
      jackMotor.setSelectedSensorPosition((int)jackInchesToEncoderTicks(0), 0, 30);
      return true;
    } else {
      reverseLimitSwitchLastPressed = false;
      return false;
    }
  }

  public boolean isHomed() {
    return isHomed;
  }
  
  public void resetHomed(){
    this.isHomed = false;
  }

  public String periodicOutput() {
    // return "" + jackMotor.getSelectedSensorPosition();
    return "Upper: " + (isUpperLimitSwitchPressed() ? "Yes" : "No ") + "Lower: " + (isLowerLimitSwitchPressed() ? "Yes" : "No ");
    // return "";
  }
}
