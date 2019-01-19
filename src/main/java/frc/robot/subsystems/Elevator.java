/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorJoystickCommand;

public class Elevator extends Subsystem {
  
  private double targetPosition = 0;
  private boolean isHomed = false;
  private TalonSRX elevatorMotor;
  private double homingSpeed = -3;
  private int elevatorThreshold = 500; //Encoder Ticks
  private boolean isHoldingPosition = false;

  private int elevatorFirstSpaceshipPosition = 100000; //EncoderTicks

  private ElevatorState state;

  private enum ElevatorState {
    MOVING,
    HOMING,
    HOLDING,
    IDLE
  }

  public Elevator() {
    elevatorMotor = new TalonSRX(Constants.ELEVATOR_MOTOR_ID);
    state = ElevatorState.IDLE;
    elevatorMotor.overrideLimitSwitchesEnable(true);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ElevatorJoystickCommand());
  }

  public int getElevatorFirstSpaceshipPosition() {return elevatorFirstSpaceshipPosition;}

  //cm
  public void setTargetPosition(double targetPosition) {
    this.targetPosition = targetPosition;
  }

  public void moveToPosition() {
    elevatorMotor.set(ControlMode.MotionMagic, targetPosition);
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

  public void setHomed(boolean isHomed) {this.isHomed = isHomed;}

  public boolean isHomed() {return isHomed;}

  public void setIsHoldingPosition(boolean isHoldingPosition) {this.isHoldingPosition = isHoldingPosition;}

  public boolean isHoldingPosition() {return isHoldingPosition;}

  public void setSensorValue(int sensorPos) {elevatorMotor.setSelectedSensorPosition(sensorPos);}

  public int getSensorPosition() {return elevatorMotor.getSelectedSensorPosition();}

  public double getTargetPosition() {return targetPosition;}

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
