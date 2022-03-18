// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import edu.wpi.first.wpilibj.Compressor;

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

  public void manualControl(DoubleSupplier supplierSpeed) {
    var speed = supplierSpeed.getAsDouble();
    _intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void manualControl(double speed) {
    _intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  // public void openIntake(DoubleSolenoid _intakeLeft, DoubleSolenoid
  // _intakeRight)
  // {
  // _intakeLeft.set(Value.kForward);
  // _intakeRight.set(Value.kForward);
  // }

  // public void closeIntake(DoubleSolenoid _intakeLeft, DoubleSolenoid
  // _intakeRight)
  // {
  // _intakeLeft.set(Value.kReverse);
  // _intakeRight.set(Value.kReverse);
  // }

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
