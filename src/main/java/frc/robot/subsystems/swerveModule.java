/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class swerveModule extends SubsystemBase {

  public double currentPosition;
  private TalonSRX steerMotor;
  private CANSparkMax driveMotor;
  private static final double RAMP_RATE = 1.5;

  // Integrated Drive motor encoder in Spark Max/Neo
  private RelativeEncoder driveMotorEncoder;

  private static final double STEER_P = 3.0, STEER_I = 0.0, STEER_D = 0.1;
  private static final int STATUS_FRAME_PERIOD = 5;

  public double encoderCountPerRotation = 1024;

  public swerveModule(int steerNum, int driveNum, boolean invertDrive, boolean invertSteer) {

    // Create and configure a new Drive motor
    driveMotor = new CANSparkMax(driveNum, MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(invertDrive);
    driveMotor.setOpenLoopRampRate(RAMP_RATE);
    driveMotor.setIdleMode(IdleMode.kCoast);

    // Create and configure a new Steering motor
    steerMotor = new TalonSRX(steerNum);
    steerMotor.configFactoryDefault();
    steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    steerMotor.config_kP(0, STEER_P, 0);
    steerMotor.config_kI(0, STEER_I, 0);
    steerMotor.config_kD(0, STEER_D, 0);
    steerMotor.config_IntegralZone(0, 100, 0);
    steerMotor.configAllowableClosedloopError(0, 5, 0);
    steerMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
    steerMotor.setInverted(invertSteer);
    steerMotor.setSensorPhase(true);

    // Create the built in motor encoders
    driveMotorEncoder = driveMotor.getEncoder();

    driveMotorEncoder.setPosition(0);

  }

  public void setSwerve(double angle, double speed) {
    double currentPosition = steerMotor.getSelectedSensorPosition(0);
    double currentAngle = (currentPosition * 360.0 / this.encoderCountPerRotation) % 360.0;
    double targetAngle = angle;
    double deltaDegrees = targetAngle - currentAngle;
    // If we need to turn more than 180 degrees, it's faster to turn in the opposite
    // direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }

    // If we need to turn more than 90 degrees, we can reverse the wheel direction
    // instead and only rotate by the complement
    if (Math.abs(deltaDegrees) > 90.0) {
      deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
      speed = -speed;
    }
    // Add change in position to current position
    double targetPosition = currentPosition + ((deltaDegrees / 360) * encoderCountPerRotation);

    driveMotor.set(speed);
    steerMotor.set(ControlMode.Position, targetPosition);
  }

  // Get the built in Spark/Neo Drive motor encoder position. Value is in motor
  // revolutions.
  public double getDriveEncoder() {
    return driveMotorEncoder.getPosition();
  }

  // Set the position value of the Spark/Neo Drive motor encoder position.
  // Position is in motor revolutions.
  public void setDriveEncoder(double position) {
    driveMotorEncoder.setPosition(position);
  }

  // Set the drive motor speed from -1 to 1
  public void setDriveSpeed(double speed) {
    driveMotor.set(speed);
  }

  // Get the drive motor speed.
  public double getDriveSpeed() {
    return driveMotor.get();
  }

  public double getSteerEncoder() {
    double curPosition = steerMotor.getSelectedSensorPosition(0);
    return curPosition;
  }

  public void stopDriveMotor() {
    driveMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
