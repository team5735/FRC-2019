/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class BasicSolenoid extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid doubleSolenoid;
  private Compressor compressor;

  public BasicSolenoid() {
    doubleSolenoid = new DoubleSolenoid(0, 1, 0); // PCM, forward id, reverse id
    compressor = new Compressor(0);
    compressor.setClosedLoopControl(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void open() {
    doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void close() {
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void togglePosition() {
    if (doubleSolenoid.get() == DoubleSolenoid.Value.kForward) {
      close();
    } else {
      open();
    }
  }

  public void startCompressing() {
    compressor.start();
  }
  
  public void stopCompressing() {
    compressor.stop();
  }

  public void toggleCompressing() {
    if (compressor.enabled()) {
      stopCompressing();
    } else {
      startCompressing();
    }
  }
}
