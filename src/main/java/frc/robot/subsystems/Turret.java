// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Turret extends SubsystemBase {

  public static Turret instance;
  private CANSparkMax _turretMotor;
  private RelativeEncoder _turretEncoder;
  private Servo leftHoodServo;
  private Servo rightHoodServo;

  public static final double WPILIB_MIN_SERVO_ANGLE = 0.0; //degrees
  public static final double WPILIB_MAX_SERVO_ANGLE = 360.0; //degrees
  private static final double TIME_TO_SERVO_FULL_EXTENSION = 2.60; //Avg time to move from retract to extend
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

  final double ANGULAR_P = 0.08;//0.15
  final double ANGULAR_I = 0.011;
  final double ANGULAR_D = 0.00;

  PIDController _turretPIDController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

  private double _targetYaw;
  private boolean _hasTarget;

  private double _encoderZero;

  /** Creates a new Turret. */
  private Turret() {    
    CommandScheduler.getInstance().registerSubsystem(this);
    leftHoodServo = new Servo(0);
    leftHoodServo.setBounds(HOOD_MAX_PWM, CENTER_SERVO_PWM + SERVO_DEADBAND,
        CENTER_SERVO_PWM, CENTER_SERVO_PWM - SERVO_DEADBAND, HOOD_MIN_PWM);
    rightHoodServo = new Servo(1);
    rightHoodServo.setBounds(HOOD_MAX_PWM, CENTER_SERVO_PWM + SERVO_DEADBAND,
        CENTER_SERVO_PWM, CENTER_SERVO_PWM - SERVO_DEADBAND, HOOD_MIN_PWM);
    // Lower turret to 0 as starting position for Auto
    lowerTurretAngle();
    setupTurretMotor();
  }

  public static Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }
    return instance;
  }

  public void raiseTurretAngle() {
    leftHoodServo.setAngle(60);
    rightHoodServo.setAngle(60);
  }

  public void lowerTurretAngle() {
    leftHoodServo.setAngle(0);
    rightHoodServo.setAngle(0);
  }

  public void setTurretPosition(double position) {
    leftHoodServo.setAngle(position);
    rightHoodServo.setAngle(position);
  }

  public void setTurretAngle(double angle) {
    leftHoodServo.setAngle(angle);
    rightHoodServo.setAngle(angle);
  }

  public boolean getTurretHoodPosition(){
    if (leftHoodServo.getAngle()<49) {
      return true;
    }
    else {
      return false;
    }

  }

  public void rotateTurret(DoubleSupplier speedSupplier, double TURRET_DEADZONE, boolean hasTarget, double targetYaw) {

    double joyStickSpeed = speedSupplier.getAsDouble();
    double speed;
    this._hasTarget = hasTarget;
    this._targetYaw = targetYaw;
    //System.out.println("Turret 'speed': " + speed);
    
    if (this._hasTarget){
      speed = _turretPIDController.calculate(this._targetYaw, 0);
    }
    else{
      //deadzone clause, deadzone is 0.12 (or not, check TurretCommand.java)
      if(Math.abs(joyStickSpeed) > TURRET_DEADZONE) {
        speed = joyStickSpeed*.75;
      }
      else {
        speed = 0;
      }
    }

    _turretMotor.set(speed);

    //untested and unimplemented code to software limit the turret
    // if(speed > 0 && this._turretEncoder.getPosition() > 0) {
    //   _turretMotor.set(0);
    // }

  }

  public void stopTurretRotation() {
    _turretMotor.set(0);
  }

  
  private void setupTurretMotor() {
    _turretMotor = new CANSparkMax(Constants.Turret, MotorType.kBrushless);
    _turretMotor.restoreFactoryDefaults();
    _turretMotor.setIdleMode(IdleMode.kBrake);

    // Encoder object created to display position values
    _turretEncoder = _turretMotor.getEncoder();
    _encoderZero = _turretEncoder.getPosition();

  }

  public void resettingEncoder() {
    _turretEncoder.setPosition(_encoderZero);
  }

  public void setTurretPosition(double speed, double position) {

    double delta = this.getTurretPosition();
    //double speed = _turretPIDController.calculate(delta, position);
    if(position < delta) {
    this._turretMotor.set(speed);
    }
    else {
      this._turretMotor.set(0);
    }

  }

  public double getTurretPosition() {
    return _turretEncoder.getPosition();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Hood Servo", this.leftHoodServo.getAngle());
    SmartDashboard.putNumber("Right Hood Servo", this.rightHoodServo.getAngle());
    SmartDashboard.putNumber("Turret Encoder Position", this._turretEncoder.getPosition());
  }
}
