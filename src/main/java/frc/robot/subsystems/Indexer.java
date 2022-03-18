// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

  private static Indexer instance;

  private TalonSRX _indexMotor;

  private static final double MAX_INDEXER_SPEED = 1;

  /** Creates a new Indexer. */
  private Indexer() {
    CommandScheduler.getInstance().registerSubsystem(this);

    _indexMotor = new TalonSRX(Constants.INDEXER_ID);
    _indexMotor.configFactoryDefault();
  }

  // Public Methods
  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }

  public void manualControl(DoubleSupplier supplierSpeed) {
    var speed = supplierSpeed.getAsDouble();
    _indexMotor.set(ControlMode.PercentOutput, speed);
  }

  public void manualControl(double speed) {
    _indexMotor.set(ControlMode.PercentOutput, speed);
  }

  public void autoControl(double speed) {
    _indexMotor.set(ControlMode.PercentOutput, speed);
  }

  public void forward() {
    this.manualControl(MAX_INDEXER_SPEED);
  }

  public void reverse() {
    this.manualControl(-MAX_INDEXER_SPEED);
  }

  public void stop() {
    this.manualControl(0);
  }
}
