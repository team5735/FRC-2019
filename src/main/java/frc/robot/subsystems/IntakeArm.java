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
  private static final double kP = 0.1;
  private static final double kI = 0.1;
  private static final double kD = 0;
  private static final double kF = 0;
  private static final double kA = 0; // Arbitrary feed forward (talon directly adds this % out to counteract gravity)

  private static final double ARM_LENGTH= 15.25; //inches
  private static final double HEIGHT_OF_GROUND = 10; //inches

  private static final int THRESHOLD = 2;

  private final double UPPER_LIMIT = ARM_LENGTH + HEIGHT_OF_GROUND - 0.0001;
  private final double LOWER_LIMIT = intakeArmEncoderTicksToInches(-4096. / 360. * 30.) + 0.0001; // 69

  private double targetPosition = 0;

  public IntakeArm() {
    intakeArmMotor = new TalonSRX(Constants.INTAKE_ARM_MOTOR_ID);
    intakeArmMotor.configFactoryDefault();

    intakeArmMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    intakeArmMotor.setInverted(false);
    intakeArmMotor.setSensorPhase(true);
    intakeArmMotor.overrideLimitSwitchesEnable(true);
    resetSensorPosition();

    // Set motion magic parameters
    intakeArmMotor.configMotionCruiseVelocity(400);
    intakeArmMotor.configMotionAcceleration(400);

    // Set main motor PID values
    intakeArmMotor.selectProfileSlot(0, 0);
    intakeArmMotor.config_kP(0, kP);
    intakeArmMotor.config_kI(0, kI);
    intakeArmMotor.config_kD(0, kD);
    intakeArmMotor.config_kF(0, kF);

    intakeArmFollower = new TalonSRX(Constants.INTAKE_ARM_MOTOR_FOLLOWER_ID);
    intakeArmFollower.configFactoryDefault();
    intakeArmFollower.setInverted(true);
    intakeArmFollower.follow(intakeArmMotor);


    spinnyMotor = new TalonSRX(Constants.INTAKE_ARM_SPINNER_MOTOR_ID);
    spinnyMotor.configFactoryDefault();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    
    setDefaultCommand(new IntakeArmManual());
  }

  public void setTargetPosition(double targetPosition) {
    if (targetPosition < LOWER_LIMIT) {
      this.targetPosition = LOWER_LIMIT;
    } else if (targetPosition > UPPER_LIMIT) {
      this.targetPosition = UPPER_LIMIT;
    } else {
      this.targetPosition = targetPosition;
    }
  }

  public boolean isInPosition() {
    double positionError = Math.abs(intakeArmEncoderTicksToInches(getSensorPosition()) - this.targetPosition);
    return positionError < THRESHOLD;
  }

  public int getSensorPosition() {
    return intakeArmMotor.getSelectedSensorPosition();
  }

  public void updateMotionMagic() {
    intakeArmMotor.set(ControlMode.MotionMagic, intakeArmInchesToEncoderTicks(targetPosition));
  }

  public void updatePercentOutputOnArm(double value) {
    intakeArmMotor.set(ControlMode.PercentOutput, value);
  }

  public void updatePercentOutputOnSpinner(double value) {
    spinnyMotor.set(ControlMode.PercentOutput, value);
  }

  public void resetSensorPosition() {
    intakeArmMotor.setSelectedSensorPosition(0, 0, 30);
  }

  //straight up is zero
  public double intakeArmEncoderTicksToInches(double encoderTicks) {
    return Math.cos(encoderTicks / 4096. * 2. * Math.PI) * ARM_LENGTH + HEIGHT_OF_GROUND;
  }

  public double intakeArmInchesToEncoderTicks(double inches) {
    return Math.acos((inches - HEIGHT_OF_GROUND) / ARM_LENGTH) / 2. / Math.PI * 4096.;
  }

  public boolean isUpperLimitSwitchPressed() {
    return intakeArmMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean isLowerLimitSwitchPressed() {
    return intakeArmMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public String periodicOutput() {
    return "" + intakeArmMotor.getSelectedSensorPosition();
    // return "Upper: " + (isUpperLimitSwitchPressed() ? "Yes" : "No ") + "Lower: " + (isLowerLimitSwitchPressed() ? "Yes" : "No ");
  }
}
