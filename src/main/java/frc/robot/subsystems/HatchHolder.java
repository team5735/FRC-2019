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
public class HatchHolder extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private DoubleSolenoid clawSolenoid, extentionSolenoid;
  private Compressor compressor;

  public HatchHolder() {
    clawSolenoid = new DoubleSolenoid(1, 1, 0); // PCM, forward id, reverse id
    extentionSolenoid = new DoubleSolenoid(1, 2, 3);
    compressor = new Compressor(1);
    compressor.setClosedLoopControl(false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void openClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void closeClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void toggleClaw() {
    if (clawSolenoid.get() == DoubleSolenoid.Value.kForward) {
      closeClaw();
    } else {
      openClaw();
    }
  }

  public void extendExtentention() {
    extentionSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void unextendExtentention() {
    extentionSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void toggleExtentention() {
    if (extentionSolenoid.get() == DoubleSolenoid.Value.kForward) {
      unextendExtentention();
    } else {
      extendExtentention();
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

  public String periodicOutput() {
    return "";
  }
}
