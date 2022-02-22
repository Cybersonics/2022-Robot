// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {

  private static Pneumatics instance;

  private DoubleSolenoid _intakeLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.L_INTAKE_OUT, Constants.L_INTAKE_IN);
  private DoubleSolenoid _intakeRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.R_INTAKE_OUT, Constants.R_INTAKE_IN);
  private DoubleSolenoid _climberLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.L_CLIMBER_OUT, Constants.L_CLIMBER_IN);
  private DoubleSolenoid _climberRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.R_CLIMBER_OUT, Constants.R_CLIMBER_IN);
  
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  /** Creates a new Pnuematics. */
  private Pneumatics() {
    pcmCompressor.enableDigital();
    
    _intakeLeft.set(Value.kOff);
    _intakeRight.set(Value.kOff);

    _climberLeft.set(Value.kOff);
    _climberRight.set(Value.kOff);
  }

  public static Pneumatics getInstance() {
    if (instance == null) {
      instance = new Pneumatics();
    }
    return instance;
  }

  public void intakeOut() {
    if (_intakeLeft.get() == Value.kOff || _intakeLeft.get() == Value.kForward) {
      _intakeLeft.toggle(); 
    }
    if (_intakeRight.get() == Value.kOff || _intakeRight.get() == Value.kForward) {
      _intakeRight.toggle();
    }
  }

  public void intakeIn() {
    if (_intakeLeft.get() == Value.kOff || _intakeLeft.get() == Value.kReverse) {
      _intakeLeft.toggle(); 
    }
    if (_intakeRight.get() == Value.kOff || _intakeRight.get() == Value.kReverse) {
      _intakeRight.toggle();
    }
  }

  public void climberUp() {
    if (_intakeLeft.get() == Value.kOff || _intakeLeft.get() == Value.kReverse) {
      _intakeLeft.toggle(); 
    }
    if (_intakeRight.get() == Value.kOff || _intakeRight.get() == Value.kReverse) {
      _intakeRight.toggle();
    }
  }

  public void climberDown() {
    if (_climberLeft.get() == Value.kOff || _climberLeft.get() == Value.kReverse) {
      _climberLeft.toggle(); 
    }
    if (_climberRight.get() == Value.kOff || _climberRight.get() == Value.kReverse) {
      _climberRight.toggle();
    }
  }
}
