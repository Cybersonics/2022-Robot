// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.DriveLocation;
import frc.robot.utility.SwerveDrive;

public class Drive extends SubsystemBase {

  private final double robotWidth;
  private final double robotLength;

  private SwerveDrive<CANSparkMax, TalonSRX> FL_Drive;
  private SwerveDrive<CANSparkMax, TalonSRX> FR_Drive;
  private SwerveDrive<CANSparkMax, TalonSRX> BL_Drive;
  private SwerveDrive<CANSparkMax, TalonSRX> BR_Drive;

  private static final double STEER_P = 10.0, STEER_I = 0.02, STEER_D = 0.0;
  private static final int STATUS_FRAME_PERIOD = 5;

  /**
   * Constructor to create class responsible for converting joystick interaction
   * to output on drives.
   * 
   * @param width  width of the robot in inches
   * @param length length of the robot in inches
   */
  public Drive(double width, double length) {
    this.robotWidth = width;
    this.robotLength = length;

    this.FL_Drive = new SwerveDrive.SwerveDriveBuilder<CANSparkMax, TalonSRX>(DriveLocation.FrontLeft, 4.0)
        .DriveMotor(buildDriveMotor(Constants.FL_Drive_Id))
        .SteerMotor(buildSteerMotor(Constants.FL_Steer_Id))
        .Encoder(null, 1024.0)
        .Build();

    this.FR_Drive = new SwerveDrive.SwerveDriveBuilder<CANSparkMax, TalonSRX>(DriveLocation.FrontRight, 4.0)
        .DriveMotor(buildDriveMotor(Constants.FR_Drive_Id))
        .SteerMotor(buildSteerMotor(Constants.FR_Steer_Id))
        .Encoder(null, 1024.0)
        .Build();

    this.BL_Drive = new SwerveDrive.SwerveDriveBuilder<CANSparkMax, TalonSRX>(DriveLocation.BackLeft, 4.0)
        .DriveMotor(buildDriveMotor(Constants.BL_Drive_Id))
        .SteerMotor(buildSteerMotor(Constants.BL_Steer_Id))
        .Encoder(null, 1024.0)
        .Build();

    this.BR_Drive = new SwerveDrive.SwerveDriveBuilder<CANSparkMax, TalonSRX>(DriveLocation.BackRight, 4.0)
        .DriveMotor(buildDriveMotor(Constants.BR_Drive_Id))
        .SteerMotor(buildSteerMotor(Constants.BR_Steer_Id))
        .Encoder(null, 1024.0)
        .Build();
  }

  private CANSparkMax buildDriveMotor(int driveId) {
    var drive = new CANSparkMax(driveId, MotorType.kBrushless);
    drive.restoreFactoryDefaults();
    drive.setIdleMode(IdleMode.kBrake);
    drive.setOpenLoopRampRate(0.125);
    drive.setSmartCurrentLimit(60);

    return drive;
  }

  private TalonSRX buildSteerMotor(int steerId) {
    var steer = new TalonSRX(steerId);
    steer.configFactoryDefault();
    steer.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    steer.setInverted(false);
    steer.config_kP(0, STEER_P, 0);
    steer.config_kI(0, STEER_I, 0);
    steer.config_kD(0, STEER_D, 0);
    steer.config_IntegralZone(0, 100, 0);
    steer.configAllowableClosedloopError(0, 5, 0);
    steer.setNeutralMode(NeutralMode.Brake);
    steer.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

    return steer;
  }

  public void processInput(double forward, double strafe, double omega) {
    double omegaL2 = omega * (this.robotWidth / 2.0);
    double omegaW2 = omega * (this.robotLength / 2.0);

    // Compute the constants used later for calculating speeds and angles
    double A = strafe - omegaL2;
    double B = strafe + omegaL2;
    double C = forward - omegaW2;
    double D = forward + omegaW2;

    // Compute the drive motor speeds
    double speedLF = speed(B, D);
    double speedLR = speed(A, D);
    double speedRF = speed(B, C);
    double speedRR = speed(A, C);

    // Angles for the steering motors
    // When drives are calibrated for zero position on encoders
    // They are at 90 degrees to the front of the robot.
    // Subtract and add 90 degrees to steering calculation to offset for initial
    // position/calibration of drives.
    double angleLF = angle(B, D) - 90;
    double angleLR = angle(A, D) + 90;
    double angleRF = angle(B, C) - 90;
    double angleRR = angle(A, C) + 90;

    // Compute the maximum speed so that we can scale all the speeds to the range
    // [0.0, 1.0]
    double maxSpeed = Collections.max(Arrays.asList(speedLF, speedLR, speedRF, speedRR, 1.0));

    // Set each swerve module, scaling the drive speeds by the maximum speed
    this.FL_Drive.setAngleAndSpeed(angleLF, speedLF / maxSpeed);
    this.BL_Drive.setAngleAndSpeed(angleLR, speedLR / maxSpeed);
    this.FR_Drive.setAngleAndSpeed(angleRF, speedRF / maxSpeed);
    this.BR_Drive.setAngleAndSpeed(angleRR, speedRR / maxSpeed);
  }

  private double speed(double val1, double val2) {
    return Math.hypot(val1, val2);
  }

  private double angle(double val1, double val2) {
    return Math.toDegrees(Math.atan2(val1, val2));
  }
}
