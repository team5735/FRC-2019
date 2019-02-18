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
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.intakeArm.IntakeArmHoldPosition;
import frc.robot.commands.intakeArm.IntakeArmJoystick;
import frc.robot.commands.intakeArm.IntakeArmManual;
import frc.robot.Constants;

/**
 * When zeroing the straight up is zero;
 */
public class IntakeArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX intakeArmMotor;
  private TalonSRX intakeArmFollower;
  private TalonSRX spinnyMotor;

  // PID Values
  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double kF = 1023. / 1120.;
  private static final double kA = 0; // Arbitrary feed forward (talon directly adds this % out to counteract gravity)

  private static final double ARM_LENGTH = 15.25; //inches
  private static final double HEIGHT_OFF_GROUND = 10; //inches
  // private static final double OFFSET_TO_STRAIGHT_UP = 30; //degrees

  private static final double MAX_FEEDFORWARD = 0.2;

  private static final int THRESHOLD = 2;

  public class Angle {
    public static final double ZERO = 0, INSIDE = 20, VERY_INSIDE = 10, SAFE = 35, INTAKE = 60, MAX_ANGLE = 130, READY = 30, OFFSET = 40;

    private double value;

    public Angle(double value) {
      this.value = value;
    }

    public double getValue() {
      return this.value;
    }
  }
  
  // private final double UPPER_LIMIT = 150, LOWER_LIMIT = 0;

  // private final double UPPER_LIMIT = ARM_LENGTH + HEIGHT_OFF_GROUND - 0.0001;
  // private final double LOWER_LIMIT = 10 + 0.0001; // 69
  // private final double LOWER_LIMIT = intakeArmEncoderTicksToInches(-4096. / 360. * 30.) + 0.0001; // 69

  private double targetAngle = 0;

  public IntakeArm() {
    intakeArmMotor = new TalonSRX(Constants.INTAKE_ARM_MOTOR_ID);
    intakeArmMotor.configFactoryDefault();

    intakeArmMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    intakeArmMotor.setInverted(true);
    intakeArmMotor.setSensorPhase(true);
    intakeArmMotor.overrideLimitSwitchesEnable(true);
    resetSensorPosition();

    // Set motion magic parameters
    intakeArmMotor.configMotionCruiseVelocity(200);
    intakeArmMotor.configMotionAcceleration(200);

    // Set main motor PID values
    intakeArmMotor.selectProfileSlot(0, 0);
    intakeArmMotor.config_kP(0, kP);
    intakeArmMotor.config_kI(0, kI);
    intakeArmMotor.config_kD(0, kD);
    intakeArmMotor.config_kF(0, kF);

    intakeArmFollower = new TalonSRX(Constants.INTAKE_ARM_MOTOR_FOLLOWER_ID);
    intakeArmFollower.configFactoryDefault();
    intakeArmFollower.setInverted(false);
    intakeArmFollower.follow(intakeArmMotor);

    intakeArmMotor.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX,
						LimitSwitchNormal.NormallyOpen, intakeArmFollower.getDeviceID(), 10);

    spinnyMotor = new TalonSRX(Constants.INTAKE_ARM_SPINNER_MOTOR_ID);
    spinnyMotor.configFactoryDefault();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new IntakeArmManual());
    setDefaultCommand(new IntakeArmJoystick());
  }

  public void setTargetAngle(double targetAngle) {
    if (targetAngle < Angle.ZERO) {
      this.targetAngle = Angle.ZERO;
    } else if (targetAngle > Angle.MAX_ANGLE) {
      this.targetAngle = Angle.MAX_ANGLE;
    } else {
      this.targetAngle = targetAngle;
    }
  }

  public boolean isInPosition() {
    double positionError = Math.abs(getSensorPosition() - degreesToEncoderTicks(this.targetAngle));
    return positionError < THRESHOLD;
  }

  public int getSensorPosition() {
    return intakeArmMotor.getSelectedSensorPosition();
  }

  public void updatePosition() {
    if(isUpperLimitSwitchPressed()) {
      resetSensorPosition();
    }
    intakeArmMotor.set(ControlMode.Position, degreesToEncoderTicks(targetAngle));
  }

  public void updateMotionMagic() {
    if(isUpperLimitSwitchPressed()) {
      resetSensorPosition();
    }
    intakeArmMotor.set(ControlMode.MotionMagic, degreesToEncoderTicks(targetAngle), DemandType.ArbitraryFeedForward, 0); // -MAX_FEEDFORWARD*Math.sin(targetAngle - Angle.OFFSET)
  }

  public void updatePercentOutputOnArm(double value) {
    if(isUpperLimitSwitchPressed()) {
      resetSensorPosition();
    }
    intakeArmMotor.set(ControlMode.PercentOutput, value);
  }

  public void updatePercentOutputOnSpinner(double value) {
    spinnyMotor.set(ControlMode.PercentOutput, value);
  }

  public void resetSensorPosition() {
    intakeArmMotor.setSelectedSensorPosition((int)degreesToEncoderTicks(Angle.MAX_ANGLE), 0, 30);
  }

  public boolean isUpperLimitSwitchPressed() {
    return intakeArmMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean isLowerLimitSwitchPressed() {
    return intakeArmMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public double getPercentOutput() {
    return intakeArmMotor.getMotorOutputPercent();
  }

  // public String periodicOutput() {
  //   return "" + intakeArmMotor.getSelectedSensorPosition();
  //   // return "Upper: " + (isUpperLimitSwitchPressed() ? "Yes" : "No ") + "Lower: " + (isLowerLimitSwitchPressed() ? "Yes" : "No ");
  // }

  public double getCurrentDegrees() {
    return encoderTicksToDegrees(getSensorPosition());
  }

  public double getArmVelocityInEncoderTicks() {
    return intakeArmMotor.getSelectedSensorVelocity();
  }

  public double getTargetDegress() {
    return targetAngle;
  }

  public double intakeArmInchesToEncoderTicks(double inches) {
    return Math.acos((inches - HEIGHT_OFF_GROUND) / ARM_LENGTH) / 2. / Math.PI * 4096. + Angle.OFFSET / 360. * 4096.;
  }

  // public double intakeArmEncoderTicksToInches(double encoderTicks) {
  //   return Math.cos(encoderTicks / 4096. * 2. * Math.PI) * ARM_LENGTH + HEIGHT_OFF_GROUND;
  // }

  public double inchesToDegrees (double inches) {
    return encoderTicksToDegrees(intakeArmInchesToEncoderTicks(inches));
  }

  public double encoderTicksToDegrees (double encoderTicks) {
    return encoderTicks / 4096.* 360. * 24 / 84;
  }

  public double degreesToEncoderTicks (double degrees) {
    return degrees /360. * 4096. / 24 * 84;
  }

  public boolean isArmSafe (double minSafeAngle) {
    return ((targetAngle < minSafeAngle) && (getCurrentDegrees() > minSafeAngle));
  }
}
