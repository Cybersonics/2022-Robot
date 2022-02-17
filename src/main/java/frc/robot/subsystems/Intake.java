// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static Intake instance;

  private TalonSRX _intakeMotor;

  private static final double MAX_INDEXER_SPEED = .5;

  /** Creates a new Intake. */
  private Intake() {
    CommandScheduler.getInstance().registerSubsystem(this);

    _intakeMotor = new TalonSRX(Constants.INTAKER_ID);
    _intakeMotor.configFactoryDefault();
  }

  // Public Methods
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  public void manualControl(double speed) {
    _intakeMotor.set(ControlMode.PercentOutput, speed);
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
