// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.DriveLocation;
import frc.robot.utility.SwerveDrive;

public class Drive extends SubsystemBase {

  private SwerveDrive FL_Drive;
  private SwerveDrive FR_Drive;
  private SwerveDrive BL_Drive;
  private SwerveDrive BR_Drive;
  
  public Drive() {
    this.FL_Drive = new SwerveDrive.SwerveDriveBuilder(DriveLocation.FrontLeft)
    .DriveMotor(new CANSparkMax(1, MotorType.kBrushless))
    .SteerMotor(new TalonSRX(11))
    .Encoder(null)
    .Build();

    this.FR_Drive = new SwerveDrive.SwerveDriveBuilder(DriveLocation.FrontRight)
    .DriveMotor(new CANSparkMax(2, MotorType.kBrushless))
    .SteerMotor(new TalonSRX(12))
    .Encoder(null)
    .Build();

    this.BL_Drive = new SwerveDrive.SwerveDriveBuilder(DriveLocation.BackLeft)
    .DriveMotor(new CANSparkMax(3, MotorType.kBrushless))
    .SteerMotor(new TalonSRX(13))
    .Encoder(null)
    .Build();

    this.BR_Drive = new SwerveDrive.SwerveDriveBuilder(DriveLocation.BackRight)
    .DriveMotor(new CANSparkMax(4, MotorType.kBrushless))
    .SteerMotor(new TalonSRX(14))
    .Encoder(null)
    .Build();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
