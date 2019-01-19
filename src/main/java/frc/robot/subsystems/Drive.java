/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  /**
   * Add your docs here.
   */

  private TalonSRX leftMaster, rightMaster, leftFollower, rightFollower;

  public Drive() {
    // Intert a subsystem name and PID values here
    // super("SubsystemName", 1, 2, 3);

    
    leftMaster = createTalon(1, true);
    leftFollower = createTalon(2, true, leftMaster);
    rightMaster = createTalon(1, true);
    rightFollower = createTalon(2, true, rightMaster);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public TalonSRX createTalon(int id, boolean left) {
    TalonSRX talon = new TalonSRX(id);
    talon.set(ControlMode.PercentOutput, 0.0);
    talon.setInverted(left);
    return talon;
  }

  public TalonSRX createTalon(int id, boolean left, TalonSRX master) {
    TalonSRX talon = new TalonSRX(id);
    talon.set(ControlMode.Follower, 0.0);
    talon.setInverted(left);
    talon.follow(master);
    return talon;
  }
}
