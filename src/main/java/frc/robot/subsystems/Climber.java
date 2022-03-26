// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private static Climber instance;
  public CANSparkMax _leftMotor;
  public CANSparkMax _rightMotor;

  public double MAX_SPEED = .85;

  /** Creates a new Climber. */
  private Climber() {
    setupLeftMotor();
    setupRightMotor();
  }

  // Public Methods
  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  public void releaseClimber() {
    _leftMotor.set(-MAX_SPEED);
    _rightMotor.set(-MAX_SPEED);
  }

  public void retractClimber() {
    _leftMotor.set(MAX_SPEED);
    _rightMotor.set(MAX_SPEED);
  }

  public void stop() {
    _leftMotor.set(0);
    _rightMotor.set(0);
  }

  private void setupLeftMotor() {
    _rightMotor = new CANSparkMax(Constants.L_CLIMBER_ID, MotorType.kBrushless);
    _rightMotor.restoreFactoryDefaults();
    //_rightMotor.setIdleMode(IdleMode.kCoast);
    _rightMotor.setIdleMode(IdleMode.kBrake);
  }

  private void setupRightMotor() {
    _leftMotor = new CANSparkMax(Constants.R_CLIMBER_ID, MotorType.kBrushless);
    _leftMotor.restoreFactoryDefaults();
    //_leftMotor.setIdleMode(IdleMode.kCoast);
    _leftMotor.setIdleMode(IdleMode.kBrake);
    _leftMotor.setInverted(true);
  }
}
