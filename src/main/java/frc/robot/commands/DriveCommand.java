// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;

public class DriveCommand extends CommandBase {

  private Drive _drive;
  private Joystick leftStick;
  private Joystick rightStick;
  private NavXGyro _navXGyro;

  public static final double OMEGA_SCALE = 1.0 / 45.0;// 30
  public static final double DEADZONE_LSTICK = 0.06;
  private static final double DEADZONE_RSTICK = 0.07;
  private double originHeading = 0.0;
  private double leftPow = 1;
  private double rightPow = 1;

  public DriveCommand(Drive drive, Joystick leftStick, Joystick rightStick, NavXGyro gyro) {
    this._drive = drive;
    this.leftStick = leftStick;
    this.rightStick = rightStick;
    this._navXGyro = gyro;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originHeading = _navXGyro.getZeroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double stickForward = this.leftStick.getY();
    double stickStrafe = this.leftStick.getX();
    double stickOmega = this.rightStick.getX();

    double strafe = Math.pow(Math.abs(stickStrafe), leftPow) * Math.signum(-stickStrafe);
    double forward = Math.pow(Math.abs(stickForward), leftPow) * Math.signum(stickForward);
    double omega = Math.pow(Math.abs(stickOmega), rightPow) * Math.signum(-stickOmega) * OMEGA_SCALE;

    if (Math.abs(strafe) < DEADZONE_LSTICK)
      strafe = 0.0;
    if (Math.abs(forward) < DEADZONE_LSTICK)
      forward = 0.0;
    if (Math.abs(omega) < DEADZONE_RSTICK * OMEGA_SCALE)
      omega = 0.0;
    boolean stickFieldCentric = leftStick.getTrigger();

    if (!stickFieldCentric) {
      final double originCorrection = Math.toRadians(originHeading - _navXGyro.getNavAngle());
      final double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
      strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);
      forward = temp;
    }

    // If all of the joysticks are in the deadzone, don't update the motors this makes side-to-side strafing much smoother
    boolean deadStick = false;
    if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
      deadStick = true;
    }

    this._drive.processInput(forward, strafe, omega, deadStick);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
