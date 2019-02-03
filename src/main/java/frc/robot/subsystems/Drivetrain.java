/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.DrivetrainJoystick;
import frc.robot.commands.drivetrain.DrivetrainManual;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX leftMaster, rightMaster, leftFollower, rightFollower;
  private PigeonIMU pigeonIMU;

  private static final int MAX_DRIVETRAIN_VELOCITY = 3700;

  // Curvature drive
  private double m_quickStopThreshold = 0.2;
  private double m_quickStopAlpha = 0.1;
  private double m_quickStopAccumulator = 0.0;
  private double m_deadband = 0.02;

  //PID Right
  public static final double RIGHT_kF = 1023./3600,
                            RIGHT_kP = 0.005,
                            RIGHT_kI = 0.001,
                            RIGHT_kD = 10;

  //PID Left
  public static final double LEFT_kF = 1023./3700,
                            LEFT_kP = 0.005,
                            LEFT_kI = 0.001,
                            LEFT_kD = 10;


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DrivetrainManualCommand());
  }

  public Drivetrain() {
    leftMaster = new TalonSRX(Constants.DRIVETRAIN_LEFT_MASTER_MOTOR_ID);
    leftMaster.configFactoryDefault();
    leftMaster.setSensorPhase(true);
    leftMaster.selectProfileSlot(0, 0);
    leftMaster.config_kP(0, LEFT_kP);
    leftMaster.config_kI(0, LEFT_kI);
    leftMaster.config_kD(0, LEFT_kD);
    leftMaster.config_kF(0, LEFT_kF);

    leftFollower = new TalonSRX(Constants.DRIVETRAIN_LEFT_FOLLOWER_MOTOR_ID);
    leftFollower.configFactoryDefault();
    leftFollower.follow(leftMaster);

    rightMaster = new TalonSRX(Constants.DRIVETRAIN_RIGHT_MASTER_MOTOR_ID);
    rightMaster.configFactoryDefault();
    rightMaster.setInverted(true);
    rightMaster.setSensorPhase(true);
    rightMaster.selectProfileSlot(0, 0);
    rightMaster.config_kP(0, RIGHT_kP);
    rightMaster.config_kI(0, RIGHT_kI);
    rightMaster.config_kD(0, RIGHT_kD);
    rightMaster.config_kF(0, RIGHT_kF);

    rightFollower = new TalonSRX(Constants.DRIVETRAIN_RIGHT_FOLLOWER_MOTOR_ID);
    rightFollower.configFactoryDefault();
    rightFollower.setInverted(true);
    rightFollower.follow(rightMaster);

    pigeonIMU = new PigeonIMU(rightFollower); // Motor pigeon is attachd to
  }

  public void updatePercentOutput(double leftPercent, double rightPercent) {
    leftMaster.set(ControlMode.PercentOutput, leftPercent);
    rightMaster.set(ControlMode.PercentOutput, rightPercent);

    DecimalFormat df = new DecimalFormat("#");
    df.setMaximumFractionDigits(3);

    System.out.println("L: " + df.format(leftPercent) + " ---- R:" + df.format(rightPercent));
    System.out.println("LS:" + df.format(leftMaster.getSelectedSensorVelocity()) + " ----- RS:" + df.format(rightMaster.getSelectedSensorVelocity()));
  }

  public void updateVelocity(double leftVelocity, double rightVelocity) {
    leftMaster.set(ControlMode.Velocity, leftVelocity);
    rightMaster.set(ControlMode.Velocity, rightVelocity);

    DecimalFormat df = new DecimalFormat("#");
    df.setMaximumFractionDigits(3);

    System.out.println("ERRL:" + df.format(leftMaster.getClosedLoopError()) + " ----- ERRR:" + df.format(rightMaster.getClosedLoopError()));
    System.out.println("TL:" + df.format(leftMaster.getClosedLoopTarget()) + " ----- TR:" + df.format(rightMaster.getClosedLoopTarget()));
    System.out.println("LS:" + df.format(leftMaster.getSelectedSensorVelocity()) + " ----- RS:" + df.format(rightMaster.getSelectedSensorVelocity()));
  }

  public void updateVelocityPercent(double leftPercent, double rightPercent) {
    updateVelocity(leftPercent * MAX_DRIVETRAIN_VELOCITY, rightPercent * MAX_DRIVETRAIN_VELOCITY);
  }

  public void updateArcadePercent(double drivePercent, double turnPercent, boolean isQuickTurn) {
    double[] output = curvatureDrive(drivePercent, turnPercent, isQuickTurn);
    double leftPercent = output[0];
    double rightPercent = output[1];

    updatePercentOutput(leftPercent, rightPercent);
  }

  public double[] curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    xSpeed = limit(xSpeed);
    xSpeed = applyDeadband(xSpeed, m_deadband);

    zRotation = limit(zRotation);
    zRotation = applyDeadband(zRotation, m_deadband);
    double angularPower;
    boolean overPower;

    if (isQuickTurn) {
      if (Math.abs(xSpeed) < m_quickStopThreshold) {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * limit(zRotation) * 2;
      }
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    return new double[]{leftMotorOutput, rightMotorOutput};
  }

  // ===== RobotDriveBase Methods =====

  /**
   * Limit motor values to the -1.0 to +1.0 range.
   */
  protected double limit(double value) {
    if (value > 1.0) {
      return 1.0;
    }
    if (value < -1.0) {
      return -1.0;
    }
    return value;
  }

  /**
   * Returns 0.0 if the given value is within the specified range around zero. The
   * remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value    value to clip
   * @param deadband range around zero
   */
  protected double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
