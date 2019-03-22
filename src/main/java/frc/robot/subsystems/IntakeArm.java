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
import frc.robot.commands.intakeArm.IntakeArmJoystick;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * When zeroing the straight up is zero;
 */
public class IntakeArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX intakeArmMotor;
  private TalonSRX intakeArmFollower;

  private boolean isHomed = false;

  private boolean forwardLimitSwitchLastPressed = false;
  private boolean reverseLimitSwitchLastPressed = false;

  // PID Values
  private static final double kP = 8;
  private static final double kI = 0;
  private static final double kD = 10;
  private static final double kF = 1023. / 500.;
  private static final double kA = 0; // Arbitrary feed forward (talon directly adds this % out to counteract gravity)

  private static final double ARM_LENGTH = 15; // inches
  private static final double HEIGHT_OFF_GROUND = 10; // inches
  private static final double GEAR_RATIO = 84. / 24.;

  private static final double MAX_FEEDFORWARD = 0.008; // 0.2

  public static final int THRESHOLD = 2;

  public class Angle {
    public static final double MIN_ANGLE = -150, INSIDE = -130, VERY_INSIDE = -80, SAFE = -95, INTAKE = -60,
        MAX_ANGLE = 0, READY = -86, OFFSET = -123, OFFSET_WALKER = 41;

    private double value;

    public Angle(double value) {
      this.value = value;
    }

    public double getValue() {
      return this.value;
    }
  }

  private double targetAngle = 0;

  public IntakeArm() {
    this.isHomed = false;

    intakeArmMotor = new TalonSRX(Constants.INTAKE_ARM_MOTOR_ID);
    intakeArmMotor.configFactoryDefault();

    intakeArmMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    intakeArmMotor.setInverted(false);
    intakeArmMotor.setSensorPhase(false);
    intakeArmMotor.overrideLimitSwitchesEnable(true);

    // Set motion magic parameters
    intakeArmMotor.configMotionCruiseVelocity(1000);
    intakeArmMotor.configMotionAcceleration(1000);

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
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeArmJoystick());
  }

  public void setTargetAngle(double targetAngle) {
    if (targetAngle < Angle.MIN_ANGLE) {
      this.targetAngle = Angle.MIN_ANGLE;
    } else if (targetAngle > Angle.MAX_ANGLE) {
      this.targetAngle = Angle.MAX_ANGLE;
    } else {
      this.targetAngle = targetAngle;
    }
  }

  public void resetHomed() {
    this.isHomed = false;
  }

  public void forceSetTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  public boolean isInPosition() {
    double positionError = Math.abs(getSensorPosition() - degreesToEncoderTicks(this.targetAngle));
    return positionError < THRESHOLD;
  }

  public int getSensorPosition() {
    return intakeArmMotor.getSelectedSensorPosition();
  }

  public void updatePosition() {
    isUpperLimitSwitchPressed();
    intakeArmMotor.set(ControlMode.Position, degreesToEncoderTicks(targetAngle));
  }

  public void updateMotionMagic() {
    isUpperLimitSwitchPressed();
    intakeArmMotor.set(ControlMode.MotionMagic, degreesToEncoderTicks(targetAngle), DemandType.ArbitraryFeedForward,
        -MAX_FEEDFORWARD * Math.sin(getCurrentDegrees() - Angle.OFFSET));
  }

  public void updatePercentOutputOnArm(double value) {
    isUpperLimitSwitchPressed();
    intakeArmMotor.set(ControlMode.PercentOutput, value * 0.3);
  }

  public boolean isHomed() {
    return isHomed;
  }

  // public void resetSensorPosition() {
  // System.out.println("Upper limit pressed!");
  // if (!forwardLimitSwitchLastPressed) {
  // targetAngle = Angle.MAX_ANGLE;
  // forwardLimitSwitchLastPressed = true;
  // }
  // }

  public boolean isUpperLimitSwitchPressed() {
    if (intakeArmMotor.getSensorCollection().isFwdLimitSwitchClosed()) {
      if (!forwardLimitSwitchLastPressed) {
        targetAngle = Angle.MAX_ANGLE;
        forwardLimitSwitchLastPressed = true;
      }
      isHomed = true;
      intakeArmMotor.setSelectedSensorPosition((int) degreesToEncoderTicks(Angle.MAX_ANGLE), 0, 30);

      return true;
    } else {
      forwardLimitSwitchLastPressed = false;
      return false;
    }
  }

  public boolean isLowerLimitSwitchPressed() {
    return intakeArmMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public double getPercentOutput() {
    return intakeArmMotor.getMotorOutputPercent();
  }

  // public String periodicOutput() {
  // return "" + intakeArmMotor.getSelectedSensorPosition();
  // // return "Upper: " + (isUpperLimitSwitchPressed() ? "Yes" : "No ") + "Lower:
  // " + (isLowerLimitSwitchPressed() ? "Yes" : "No ");
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

  public double intakeArmInchesToDegrees(double inches) {
    double degrees = Angle.OFFSET + Math.toDegrees(Math.acos(((inches + 2) - HEIGHT_OFF_GROUND) / ARM_LENGTH));
    if (Double.isNaN(degrees)) {
      degrees = (inches < 0) ? 0 : Angle.SAFE;
    }
    return degrees;
  }

  public double degreesToIntakeArmInches(double degrees) {
      return Math.cos(Math.toRadians(degrees - Angle.OFFSET)) * ARM_LENGTH + HEIGHT_OFF_GROUND - 2;
  }

  // public double intakeArmInchesToDegrees(double inches) {
  //   // return Math.acos((inches - HEIGHT_OFF_GROUND) / ARM_LENGTH) / 2. / Math.PI *
  //   // 4096. + Angle.OFFSET / 360. * 4096.;
  //   // return Math.abs(Math.asin((inches - HEIGHT_OFF_GROUND) / ARM_LENGTH)) / 2. /
  //   // Math.PI * 4096. + 35. / 360. * 4096.;
  //   double degrees = -Math.asin((inches - HEIGHT_OFF_GROUND) / ARM_LENGTH) / 2. / Math.PI * 360. + Angle.OFFSET_WALKER; // Walker changed Angle.OFFSET to 41
  //   // return degrees;
  //   return degrees < -90 ? -180 - degrees : degrees;
  // }

  // public double intakeArmDegreesToInches(double degrees) {
  //   // return Math.cos((encoderTicks - Angle.OFFSET / 360. * 4096.) / 4096. * 2. *
  //   // Math.PI) * ARM_LENGTH + HEIGHT_OFF_GROUND;
  //   // return Math.sin((encoderTicks - 35. / 360. * 4096.) / 4096 * 2. * Math.PI) *
  //   // ARM_LENGTH + HEIGHT_OFF_GROUND;
  //   if ((degrees + Angle.OFFSET_WALKER) < -90) {
  //     degrees = -180 - (degrees + Angle.OFFSET_WALKER);
  //   }
  //   return -(Math.sin((degrees + Angle.OFFSET_WALKER) / 360 * 2. * Math.PI) * ARM_LENGTH) + HEIGHT_OFF_GROUND;
  // }

  // public double degreesToInches(double degrees) {
  //   return intakeArmDegreesToInches(degrees);
  // }

  public double encoderTicksToDegrees(double encoderTicks) {
    return encoderTicks / 4096. * 360. * 24 / 84;
  }

  public double degreesToEncoderTicks(double degrees) {
    return degrees / 360. * 4096. / 24 * 84;
  }

  public boolean isArmSafe(double minSafeAngle) {
    return ((targetAngle < minSafeAngle) && (getCurrentDegrees() > minSafeAngle));
  }
}
