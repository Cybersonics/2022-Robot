// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Turret extends SubsystemBase {
  public static Turret instance;
  public CANSparkMax _turretMotor;

  /** Creates a new Turret. */
  private Turret() {    
    CommandScheduler.getInstance().registerSubsystem(this);
    setupTurretMotor();
  }

  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }
    return instance;
  }

  public void rotateTurret(DoubleSupplier speedSupplier) {
    double speed = speedSupplier.getAsDouble();
    _turretMotor.set(speed*.75);
  }
  
  private void setupTurretMotor() {
    _turretMotor = new CANSparkMax(Constants.Turret, MotorType.kBrushless);
    _turretMotor.restoreFactoryDefaults();
    _turretMotor.setIdleMode(IdleMode.kBrake);
  }
}
