// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Turret extends SubsystemBase {

  public static Turret instance;
  public CANSparkMax _turretMotor;
  private Servo leftHoodServo;
  private Servo rightHoodServo;

  //#region Servo Setup
  public static final double WPILIB_MIN_SERVO_ANGLE = 0.0; //degrees
  public static final double WPILIB_MAX_SERVO_ANGLE = 360.0; //degrees
  private static final double TIME_TO_SERVO_FULL_EXTENSION = 3.48; //Avg time to move from retract to extend
  private static final double PERCENT_PER_SECOND = 1.00 / TIME_TO_SERVO_FULL_EXTENSION;
  private static final double DEGREES_PER_SECOND = (WPILIB_MAX_SERVO_ANGLE - WPILIB_MIN_SERVO_ANGLE) * PERCENT_PER_SECOND;
  private static final double HOOD_MAX_POSITION = 1.0; // percent servo travel to max hood position
  private static final double HOOD_MIN_POSITION = 0.0; // percent servo travel to min hood position

  // SERVO Parameters from
  // https://s3.amazonaws.com/actuonix/Actuonix+L16+Datasheet.pdf
  private static final double MAX_SERVO_PWM = 2.0; // ms
  private static final double MIN_SERVO_PWM = 1.0; // ms
  private static final double SERVO_RANGE = MAX_SERVO_PWM - MIN_SERVO_PWM;
  private static final double CENTER_SERVO_PWM = 1.5; // ms
  private static final double SERVO_DEADBAND = 0.0; // ms - no deadband

  // pwm values in ms for the max and min angles of the shooter hood
  private static final double HOOD_MAX_PWM = MIN_SERVO_PWM + (SERVO_RANGE * HOOD_MAX_POSITION);
  private static final double HOOD_MIN_PWM = MIN_SERVO_PWM + (SERVO_RANGE * HOOD_MIN_POSITION);
  //#endregion

  //#region Turret Rotation PIDControl

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  //#endregion
  /** Creates a new Turret. */
  private Turret() {    
    CommandScheduler.getInstance().registerSubsystem(this);
    leftHoodServo = new Servo(0);
    leftHoodServo.setBounds(HOOD_MAX_PWM, CENTER_SERVO_PWM + SERVO_DEADBAND,
        CENTER_SERVO_PWM, CENTER_SERVO_PWM - SERVO_DEADBAND, HOOD_MIN_PWM);
    rightHoodServo = new Servo(1);
    rightHoodServo.setBounds(HOOD_MAX_PWM, CENTER_SERVO_PWM + SERVO_DEADBAND,
        CENTER_SERVO_PWM, CENTER_SERVO_PWM - SERVO_DEADBAND, HOOD_MIN_PWM);
    setupTurretMotor();
  }

  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }
    return instance;
  }

  public void raiseTurret() {
    leftHoodServo.setAngle(90);
    rightHoodServo.setAngle(90);
  }

  public void lowerTurret() {
    leftHoodServo.setAngle(0);
    rightHoodServo.setAngle(0);
  }

  public void rotateTurret(DoubleSupplier speedSupplier) {
    double speed = speedSupplier.getAsDouble();
    _turretMotor.set(speed*.75);
  } 

  public void setPosition(double yaw) {
    double rotationSpeed = -this.turnController.calculate(yaw, 0);
    this._turretMotor.getPIDController().setReference(rotationSpeed, ControlType.kPosition);
  }
  
  private void setupTurretMotor() {
    _turretMotor = new CANSparkMax(Constants.Turret, MotorType.kBrushless);
    _turretMotor.restoreFactoryDefaults();
    _turretMotor.setIdleMode(IdleMode.kBrake);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Hood Servo", this.leftHoodServo.getAngle());
    SmartDashboard.putNumber("Right Hood Servo", this.rightHoodServo.getAngle());
  }
}
