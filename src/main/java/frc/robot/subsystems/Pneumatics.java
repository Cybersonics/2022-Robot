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
    
    _intakeLeft.set(Value.kForward);
    _intakeRight.set(Value.kForward);

    _climberLeft.set(Value.kForward);
    _climberRight.set(Value.kForward);
  }

  public static Pneumatics getInstance() {
    if (instance == null) {
      instance = new Pneumatics();
    }
    return instance;
  }

  public void intakeIn() {
      _intakeLeft.set(Value.kForward); 
      _intakeRight.set(Value.kForward);
  }

  public void intakeOut() {
      _intakeLeft.set(Value.kReverse); 
      _intakeRight.set(Value.kReverse);
  }

  public void climberUp() {
    _climberLeft.set(Value.kForward);
    _climberRight.set(Value.kForward);
  }

  public void climberDown() {
      _climberLeft.set(Value.kReverse);
      _climberRight.set(Value.kReverse);
  }
}
